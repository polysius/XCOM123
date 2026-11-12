// XCOM123 entry point: opens a window, starts the renderer module, generates a city and its
// crowd, then runs one of three loops: interactive, a scripted benchmark, or a screenshot.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>

#include "core/BenchmarkCameraPath.h"
#include "core/BenchmarkReport.h"
#include "core/CommandLine.h"
#include "core/FlyCamera.h"
#include "core/Module_Interface.h"
#include "fluid/RiverWater.h"
#include "physics/PropField.h"
#include "render/IRenderModule.h"
#include "scene/Frustum.h"
#include "world/CityGenerator.h"
#include "world/Crowd.h"

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kBenchmarkWarmupFrames = 30;
// Benchmarks and screenshots advance the world by exactly this much per frame, so that the
// same command line always simulates and draws the same frames.
constexpr float kFixedTimeStep = 1.0f / 60.0f;
// Screenshots at a late world time simulate the crowd up to it in steps this long first. It is
// the largest step the interactive loop ever takes, so the crowd behaves the same.
constexpr float kPreRollTimeStep = 0.1f;
constexpr float kDefaultSunElevation = 24.0f;
constexpr float kSunAzimuthDegrees = 230.0f;
constexpr float kSunTurnSpeed = 10.0f;  // degrees per second while Z or X is held
constexpr int kBenchmarkFailedExitCode = 2;

double MillisecondsBetween(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// Elevation is the sun's angle above the horizon; azimuth is its compass direction, 0 meaning +Z.
glm::vec3 SunDirection(float elevationDegrees, float azimuthDegrees) {
    const float elevation = glm::radians(elevationDegrees);
    const float azimuth = glm::radians(azimuthDegrees);
    return {std::cos(elevation) * std::sin(azimuth), std::sin(elevation), std::cos(elevation) * std::cos(azimuth)};
}

void OnFramebufferResized(GLFWwindow* window, int width, int height) {
    auto* renderer = static_cast<IRenderModule*>(glfwGetWindowUserPointer(window));
    if (renderer != nullptr) {
        renderer->SetResolution(width, height);
    }
}

// Everything the frame loops share: the renderer, the generated world, and the list of people
// that are on screen this frame.
struct Engine {
    IRenderModule* renderer = nullptr;
    GLFWwindow* window = nullptr;
    CityLayout city;
    Crowd crowd;
    std::vector<NpcInstance> visibleNpcs;
    physics::PropField props;  // crates, barrels and balls under the physics engine; none unless --props is given
    fluid::RiverWater river;   // the water of the river near the props, simulated; unbuilt unless --river-sim is given
};

// Returns 0 when the window is minimised.
float WindowAspectRatio(GLFWwindow* window) {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    return height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 0.0f;
}

FrameInput MakeFrameInput(const FlyCamera& camera, float aspect, float sunElevation, float worldSeconds) {
    FrameInput frame;
    frame.view = camera.ViewMatrix();
    frame.projection = camera.ProjectionMatrix(aspect);
    frame.cameraPosition = camera.Position();
    frame.sunDirection = SunDirection(sunElevation, kSunAzimuthDegrees);
    frame.timeSeconds = worldSeconds;
    return frame;
}

// Moves the crowd forward and gathers the people the camera can see into the frame. Returns the
// time this took in milliseconds.
double UpdateCrowd(Engine& engine, float deltaSeconds, FrameInput& frame) {
    const Clock::time_point start = Clock::now();
    // The crowd simulates people near this point in the most detail.
    engine.crowd.SetFocusPoint(frame.cameraPosition);
    engine.crowd.Update(deltaSeconds);
    engine.crowd.CollectVisible(Frustum(frame.projection * frame.view), engine.visibleNpcs);
    frame.npcs = engine.visibleNpcs;
    return MillisecondsBetween(start, Clock::now());
}

// Moves the props forward and puts them into the frame.
void UpdateProps(Engine& engine, float deltaSeconds, FrameInput& frame) {
    if (engine.props.Count() == 0) {
        return;
    }
    engine.props.Update(deltaSeconds);
    frame.bodyBoxes = engine.props.Boxes();
    frame.bodySpheres = engine.props.Spheres();
}

// Moves the simulated river forward and puts its surface into the frame.
void UpdateRiver(Engine& engine, float deltaSeconds, FrameInput& frame) {
    if (!engine.river.IsBuilt()) {
        return;
    }
    engine.river.Update(deltaSeconds);
    frame.water = {.offsets = engine.river.Field(),
                   .origin = engine.river.Origin(),
                   .cellSize = engine.river.CellSize(),
                   .width = engine.river.Width(),
                   .height = engine.river.Height()};
}

// Polls window events. Returns false once the window should close.
bool KeepRunning(GLFWwindow* window) {
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    return glfwWindowShouldClose(window) == GLFW_FALSE;
}

// Shows averaged CPU, GPU and crowd timings in the window title twice a second.
class TitleBar {
public:
    TitleBar(GLFWwindow* window, std::string deviceName)
        : window(window), deviceName(std::move(deviceName)), lastUpdate(Clock::now()) {}

    void AddFrame(double crowdMs, const Engine& engine) {
        ++frames;
        crowdMsTotal += crowdMs;
        const Clock::time_point now = Clock::now();
        const double elapsedMs = MillisecondsBetween(lastUpdate, now);
        if (elapsedMs < 500.0) {
            return;
        }
        Show(elapsedMs / frames, crowdMsTotal / frames, engine);
        lastUpdate = now;
        frames = 0;
        crowdMsTotal = 0.0;
    }

private:
    void Show(double cpuMs, double crowdMs, const Engine& engine) const {
        const RenderStats stats = engine.renderer->GetStats();
        std::string gpu = "GPU timing n/a";
        if (stats.gpuTimingAvailable) {
            char text[160];
            std::snprintf(text, sizeof(text), "GPU %.2f ms (", stats.smoothedGpuFrameMs);
            gpu = text;
            for (size_t index = 0; index < stats.passes.size(); ++index) {
                std::snprintf(text, sizeof(text), "%s%.*s %.2f", index == 0 ? "" : ", ",
                              static_cast<int>(stats.passes[index].name.size()), stats.passes[index].name.data(),
                              stats.passes[index].smoothedMs);
                gpu += text;
            }
            gpu += ")";
        }

        char title[512];
        std::snprintf(title, sizeof(title),
                      "XCOM123 | %s | CPU %.2f ms (%.0f fps) | %s | scale %.0f%% | %zu people, %zu on screen | crowd %.2f ms",
                      deviceName.c_str(), cpuMs, 1000.0 / cpuMs, gpu.c_str(), stats.renderScale * 100.0f,
                      engine.crowd.AgentCount(), engine.visibleNpcs.size(), crowdMs);
        glfwSetWindowTitle(window, title);
    }

    GLFWwindow* window;
    std::string deviceName;
    Clock::time_point lastUpdate;
    int frames = 0;
    double crowdMsTotal = 0.0;
};

FlyCamera MakeStartCamera(const Options& options, const CityLayout& city) {
    if (options.camera) {
        return FlyCamera(options.camera->position, glm::radians(options.camera->yawDegrees),
                         glm::radians(options.camera->pitchDegrees));
    }
    // Above the south-west of the city, looking towards its centre.
    const glm::vec3 start(city.coreBounds.min.x * 0.55f, 70.0f, city.coreBounds.min.y * 0.55f);
    return FlyCamera(start, std::atan2(-start.x, -start.z), -0.32f);
}

int RunInteractive(Engine& engine, const Options& options) {
    FlyCamera camera = MakeStartCamera(options, engine.city);
    float sunElevation = options.sunElevationDegrees.value_or(kDefaultSunElevation);
    TitleBar title(engine.window, engine.renderer->GetDeviceName());

    const Clock::time_point startTime = Clock::now();
    Clock::time_point previousFrame = startTime;
    while (KeepRunning(engine.window)) {
        const float aspect = WindowAspectRatio(engine.window);
        if (aspect == 0.0f) {
            glfwWaitEvents();  // minimised: sleep until something happens
            previousFrame = Clock::now();
            continue;
        }

        const Clock::time_point now = Clock::now();
        // Cap the time step, so a long stall (such as dragging the window) does not teleport everyone.
        const float deltaSeconds = static_cast<float>(std::min(MillisecondsBetween(previousFrame, now) / 1000.0, 0.1));
        previousFrame = now;

        camera.Update(engine.window, deltaSeconds);
        if (glfwGetKey(engine.window, GLFW_KEY_Z) == GLFW_PRESS) {
            sunElevation -= kSunTurnSpeed * deltaSeconds;
        }
        if (glfwGetKey(engine.window, GLFW_KEY_X) == GLFW_PRESS) {
            sunElevation += kSunTurnSpeed * deltaSeconds;
        }
        sunElevation = std::clamp(sunElevation, -5.0f, 85.0f);

        const float worldSeconds = options.worldTimeSeconds.value_or(0.0f) +
                                   static_cast<float>(MillisecondsBetween(startTime, now) / 1000.0);
        FrameInput frame = MakeFrameInput(camera, aspect, sunElevation, worldSeconds);
        const double crowdMs = UpdateCrowd(engine, deltaSeconds, frame);
        UpdateProps(engine, deltaSeconds, frame);
        UpdateRiver(engine, deltaSeconds, frame);
        engine.renderer->Render(frame);
        title.AddFrame(crowdMs, engine);
    }
    return 0;
}

// Prints where each shot of the path starts as a --camera argument, so a slow view found by the
// benchmark can be looked at again with --screenshot.
void PrintShotCameras(const BenchmarkCameraPath& path) {
    std::printf("Benchmark camera path, where each shot starts (for --camera):\n");
    float shotStart = 0.0f;
    for (const BenchmarkCameraPath::Shot& shot : path.Shots()) {
        const BenchmarkCameraPath::Pose pose = path.PoseAt(shotStart);
        const glm::vec3 direction = glm::normalize(pose.target - pose.position);
        const float yawDegrees = glm::degrees(std::atan2(direction.x, direction.z));
        const float pitchDegrees = glm::degrees(std::asin(direction.y));
        std::printf("    %-16s %.1f,%.1f,%.1f,%.1f,%.1f\n", shot.name.c_str(), pose.position.x, pose.position.y,
                    pose.position.z, yawDegrees, pitchDegrees);
        shotStart += shot.share;
    }
}

// Flies the fixed camera path, measures every frame and prints the report. Returns a nonzero
// exit code when a target frame rate is given and missed, or when the run stops early.
int RunBenchmark(Engine& engine, const Options& options) {
    const BenchmarkCameraPath path(engine.city, engine.crowd);
    std::vector<std::string> shotNames;
    for (const BenchmarkCameraPath::Shot& shot : path.Shots()) {
        shotNames.push_back(shot.name);
    }
    BenchmarkRecorder recorder(shotNames);
    FlyCamera camera = MakeStartCamera(options, engine.city);
    const float sunElevation = options.sunElevationDegrees.value_or(kDefaultSunElevation);
    const float startSeconds = options.worldTimeSeconds.value_or(0.0f);
    TitleBar title(engine.window, engine.renderer->GetDeviceName());

    // The first frames warm up caches and GPU clocks at the start of the path and are not measured.
    const int totalFrames = kBenchmarkWarmupFrames + options.benchmarkFrames;
    Clock::time_point previousFrame = Clock::now();
    int frameIndex = 0;
    for (; frameIndex < totalFrames && KeepRunning(engine.window); ++frameIndex) {
        const int measuredIndex = std::max(frameIndex - kBenchmarkWarmupFrames, 0);
        const float progress = options.benchmarkFrames > 1
                                   ? static_cast<float>(measuredIndex) / static_cast<float>(options.benchmarkFrames - 1)
                                   : 0.0f;
        const BenchmarkCameraPath::Pose pose = path.PoseAt(progress);
        camera.LookAt(pose.position, pose.target);

        const float aspect = std::max(WindowAspectRatio(engine.window), 0.01f);
        const float worldSeconds = startSeconds + static_cast<float>(frameIndex) * kFixedTimeStep;
        FrameInput frame = MakeFrameInput(camera, aspect, sunElevation, worldSeconds);
        const double crowdMs = UpdateCrowd(engine, kFixedTimeStep, frame);
        UpdateProps(engine, kFixedTimeStep, frame);
        UpdateRiver(engine, kFixedTimeStep, frame);
        engine.renderer->Render(frame);
        title.AddFrame(crowdMs, engine);

        const Clock::time_point now = Clock::now();
        const double frameMs = MillisecondsBetween(previousFrame, now);
        previousFrame = now;
        if (frameIndex >= kBenchmarkWarmupFrames) {
            recorder.Record({.cpuFrameMs = frameMs,
                             .crowdMs = crowdMs,
                             .peopleOnScreen = engine.visibleNpcs.size(),
                             .shotIndex = path.ShotIndexAt(progress)},
                            engine.renderer->GetStats());
        }
    }
    PrintShotCameras(path);
    const bool passed = recorder.PrintReport(engine.crowd.AgentCount(), options.targetFps);
    const bool completed = frameIndex == totalFrames;
    if (!completed) {
        std::cerr << "Benchmark stopped after " << frameIndex << " of " << totalFrames << " frames\n";
    }
    return passed && completed ? 0 : kBenchmarkFailedExitCode;
}

// Moves the crowd and the props forward to `seconds` without drawing anything.
void PreRollWorld(Engine& engine, float seconds) {
    while (seconds > 0.0f) {
        const float step = std::min(seconds, kPreRollTimeStep);
        engine.crowd.Update(step);
        if (engine.props.Count() > 0) {
            // The physics catches up in pieces of a frame, so that none of the time is dropped.
            for (float left = step; left > 0.0f; left -= kFixedTimeStep) {
                engine.props.Update(std::min(left, kFixedTimeStep));
            }
        }
        engine.river.Update(step);  // does nothing unless the river is simulated
        seconds -= step;
    }
}

// Renders a few frames from a fixed camera at a fixed world time and saves the last one.
int RunScreenshot(Engine& engine, const Options& options) {
    const FlyCamera camera = MakeStartCamera(options, engine.city);
    const float sunElevation = options.sunElevationDegrees.value_or(kDefaultSunElevation);
    const int frames = options.screenshotFrames;
    // The last frame shows the world at `--time`; the frames before it lead up to it.
    const float renderedSeconds = static_cast<float>(frames) * kFixedTimeStep;
    const float finalSeconds = options.worldTimeSeconds.value_or(renderedSeconds);
    const float firstFrameSeconds = std::max(finalSeconds - renderedSeconds, 0.0f);
    engine.crowd.SetFocusPoint(camera.Position());
    PreRollWorld(engine, firstFrameSeconds);

    for (int frameIndex = 0; frameIndex < frames; ++frameIndex) {
        if (!KeepRunning(engine.window)) {
            return 1;
        }
        const float aspect = std::max(WindowAspectRatio(engine.window), 0.01f);
        const float worldSeconds = firstFrameSeconds + static_cast<float>(frameIndex + 1) * kFixedTimeStep;
        FrameInput frame = MakeFrameInput(camera, aspect, sunElevation, worldSeconds);
        UpdateCrowd(engine, kFixedTimeStep, frame);
        UpdateProps(engine, kFixedTimeStep, frame);
        UpdateRiver(engine, kFixedTimeStep, frame);

        const bool lastFrame = frameIndex + 1 == frames;
        if (!lastFrame) {
            engine.renderer->Render(frame);
            continue;
        }
        if (!engine.renderer->RenderAndCapture(frame, options.screenshotPath)) {
            std::cerr << "Could not save the screenshot to " << options.screenshotPath << "\n";
            return 1;
        }
        std::cout << "Saved screenshot " << options.screenshotPath << "\n";
    }
    return 0;
}

ResolutionSettings MakeResolutionSettings(const Options& options) {
    ResolutionSettings settings;
    // Screenshots are compared with each other, so they use one fixed scale unless told otherwise.
    settings.dynamic = options.dynamicResolution && !options.renderScale && !options.IsScreenshot();
    settings.fixedScale = options.renderScale.value_or(1.0f);
    settings.gpuBudgetMs = options.gpuBudgetMs;
    return settings;
}

void PrintWorldSummary(const Options& options, const Engine& engine) {
    std::cout << "City (seed " << options.seed << "): " << engine.city.boxes.size() << " boxes, "
              << engine.city.streets.edges.size() << " street segments, " << engine.city.plazas.size()
              << " squares, " << engine.crowd.AgentCount() << " people";
    if (engine.river.IsBuilt()) {
        std::cout << ", river simulated around (" << engine.river.Origin().x + 64.0f << ", " << engine.river.Origin().y + 64.0f << ")";
    }
    if (engine.props.Count() > 0) {
        std::cout << ", " << engine.props.Count() << " props above the square at (" << engine.props.DropCentre().x << ", "
                  << engine.props.DropCentre().y << ")";
    }
    std::cout << "\n";
    if (!options.IsBenchmark() && !options.IsScreenshot()) {
        std::cout << "Controls: hold the right mouse button to look, WASD to move, E/Q to go up/down,\n"
                  << "          Shift to go faster, Z/X to lower/raise the sun, Esc to quit\n";
    }
}

// Runs the engine until the window closes, the benchmark ends or the screenshot is saved.
// Returns the process exit code.
int RunEngine(const Options& options, GLFWwindow* window) {
    EngineContext engineContext;
    engineContext.window = window;
    engineContext.vsync = options.vsync;

    ModuleRegistry& modules = ModuleRegistry::Instance();
    if (!modules.CreateModule("Vulkan", engineContext)) {
        return 1;
    }
    Engine engine;
    engine.window = window;
    engine.renderer = modules.GetModule<IRenderModule>("Vulkan");
    glfwSetWindowUserPointer(window, engine.renderer);
    glfwSetFramebufferSizeCallback(window, OnFramebufferResized);
    modules.ListModules();
    engine.renderer->SetResolutionSettings(MakeResolutionSettings(options));

    // Generate the world.
    CitySettings citySettings;
    citySettings.seed = options.seed;
    if (options.satelliteCount) {
        citySettings.satelliteCount = *options.satelliteCount;
    }
    citySettings.terraces = options.terraces;
    engine.city = GenerateCity(citySettings);
    engine.renderer->UploadStaticScene(engine.city.boxes);
    engine.crowd.Spawn(engine.city, {.agentCount = options.npcCount, .seed = options.seed * 7919u + 1u});
    // Reserve room for everyone up front, so the renderer never grows buffers mid-flight.
    engine.renderer->ReserveNpcCapacity(engine.crowd.AgentCount());
    engine.visibleNpcs.reserve(engine.crowd.AgentCount());
    if (options.propCount > 0) {
        engine.props.Spawn(engine.city, {.count = options.propCount, .seed = options.seed});
    }
    if (options.riverSimulation) {
        // The river near the props, or near the middle of the city.
        const glm::vec2 near = engine.props.Count() > 0 ? engine.props.DropCentre() : glm::vec2(0.0f);
        fluid::RiverWaterSettings river;
        river.rainPerSecond = static_cast<float>(options.rainPerSecond);
        river.seed = options.seed;
        if (!engine.river.Build(engine.city, near, river)) {
            std::cerr << "The city has no water to simulate\n";
        }
    }
    PrintWorldSummary(options, engine);

    if (options.IsBenchmark()) {
        return RunBenchmark(engine, options);
    }
    if (options.IsScreenshot()) {
        return RunScreenshot(engine, options);
    }
    return RunInteractive(engine, options);
}
}  // namespace

int main(int argc, char** argv) {
    Options options;
    try {
        options = ParseCommandLine(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        PrintUsage();
        return 1;
    }
    if (options.showHelp) {
        PrintUsage();
        return 0;
    }

    if (glfwInit() == GLFW_FALSE) {
        std::cerr << "Failed to initialise GLFW\n";
        return 1;
    }
    // Vulkan draws into the window itself, so GLFW must not create an OpenGL context for it.
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(1600, 900, "XCOM123", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create a window\n";
        glfwTerminate();
        return 1;
    }

    int exitCode = 1;
    try {
        exitCode = RunEngine(options, window);
    } catch (const std::exception& error) {
        std::cerr << "Fatal error: " << error.what() << "\n";
    }
    ModuleRegistry::Instance().ShutdownAll();
    glfwDestroyWindow(window);
    glfwTerminate();
    return exitCode;
}
