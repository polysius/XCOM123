// XCOM123 entry point: opens a window, starts the renderer module, generates a city and its
// crowd, then runs one of three loops: interactive, a scripted benchmark, or a screenshot.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>

#include "core/BenchmarkCameraPath.h"
#include "core/BenchmarkReport.h"
#include "core/CommandLine.h"
#include "core/FlyCamera.h"
#include "core/FrameStatistics.h"
#include "core/StreamFlightPath.h"
#include "core/Module_Interface.h"
#include "core/JobSystem.h"
#include "demo/CharacterShowcase.h"
#include "demo/CrowdCharacters.h"
#include "editor_ui/EditorSession.h"
#include "engine_host/LiveEngineHost.h"
#include "fluid/RiverWater.h"
#include "graph_engine/SunPath.h"
#include "material/MaterialLibrary.h"
#include "physics/PropField.h"
#include "render/IRenderModule.h"
#include "scene/Frustum.h"
#include "streaming/StreamedWorld.h"
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
constexpr float kRiverPreRollSeconds = 30.0f;
// With props in the river both are run for this long before a screenshot, together, so that the props have not
// settled against water that was never moving.
constexpr float kCoupledPreRollSeconds = 60.0f;
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
    demo::CharacterShowcase characters;  // skinned, posed characters in the largest square; none unless --characters is given
    std::vector<PersonBody> peopleNearProps;  // who may push props this frame
    // The endless city of --stream. When it is there, `city` and `crowd` above stay empty: the chunks hold their own streets
    // and their own people.
    std::unique_ptr<streaming::StreamedWorld> streamed;
    demo::CrowdCharacters crowdCharacters;  // the people nearest the camera as skinned characters; only in the streamed world
    uint32_t crowdCharacterLimit = 0;
    glm::vec3 previousCameraPosition{0.0f};  // for the camera's velocity, which tells the streaming where to look ahead

    size_t PeopleCount() const { return streamed ? streamed->PeopleCount() : crowd.AgentCount(); }
};

// Returns 0 when the window is minimised.
float WindowAspectRatio(GLFWwindow* window) {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    return height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 0.0f;
}

FrameInput MakeFrameInput(const FlyCamera& camera, float aspect, const graph_engine::SunSettings& sun, float worldSeconds) {
    FrameInput frame;
    frame.view = camera.ViewMatrix();
    frame.projection = camera.ProjectionMatrix(aspect);
    frame.cameraPosition = camera.Position();
    frame.sunDirection = SunDirection(sun.elevationDegrees, sun.azimuthDegrees);
    frame.timeSeconds = worldSeconds;
    return frame;
}

// The sun as the benchmark and the screenshot start it, at the default direction.
graph_engine::SunSettings DefaultSun(const Options& options) {
    return {options.sunElevationDegrees.value_or(kDefaultSunElevation), kSunAzimuthDegrees};
}

// In the streamed world, the people within this many metres of the camera, up to a limit, are drawn as skinned characters.
constexpr float kCharacterReach = 70.0f;
constexpr uint32_t kStreamDefaultCharacters = 48;

// Moves the crowd forward and gathers the people the camera can see into the frame. Returns the
// time this took in milliseconds. `host` is the engine host of a run that has one: the people that graphs spawned are drawn too.
double UpdateCrowd(Engine& engine, float deltaSeconds, FrameInput& frame, const engine_host::LiveEngineHost* host = nullptr) {
    const Clock::time_point start = Clock::now();
    if (engine.streamed) {
        engine.streamed->UpdateCrowd(deltaSeconds, frame.cameraPosition);
        engine.streamed->CollectVisible(Frustum(frame.projection * frame.view), engine.visibleNpcs);
        engine.crowdCharacters.TakeNearest(engine.visibleNpcs, frame.cameraPosition, engine.crowdCharacterLimit,
                                           kCharacterReach, frame);
        frame.npcs = engine.visibleNpcs;
        return MillisecondsBetween(start, Clock::now());
    }
    // The crowd simulates people near this point in the most detail.
    engine.crowd.SetFocusPoint(frame.cameraPosition);
    engine.crowd.SetMovableObstacles(engine.props.Obstacles());  // where the crates are, as of the last frame
    engine.crowd.Update(deltaSeconds);
    engine.crowd.CollectVisible(Frustum(frame.projection * frame.view), engine.visibleNpcs);
    if (host != nullptr) {
        host->AppendCharacters(engine.visibleNpcs);
    }
    frame.npcs = engine.visibleNpcs;
    return MillisecondsBetween(start, Clock::now());
}

// Lets the streamed world load and release chunks around the camera. Returns the time this took in milliseconds, which is all
// the streaming costs a frame: generation happens on the manager's own threads.
double UpdateStreaming(Engine& engine, const FlyCamera& camera, float deltaSeconds) {
    if (!engine.streamed) {
        return 0.0;
    }
    const Clock::time_point start = Clock::now();
    const glm::vec3 position = camera.Position();
    const glm::vec3 velocity = deltaSeconds > 0.0f ? (position - engine.previousCameraPosition) / deltaSeconds : glm::vec3(0.0f);
    engine.previousCameraPosition = position;
    engine.streamed->Update(position, velocity);
    return MillisecondsBetween(start, Clock::now());
}

// People within this distance of a prop that they can push get a body in the physics, so that they push it as they walk.
constexpr float kPushReach = 1.2f;

void GivePeopleToProps(Engine& engine, float deltaSeconds) {
    engine.crowd.CollectPeopleAround(engine.props.PushablePositions(), kPushReach, engine.peopleNearProps);
    engine.props.SetPeople(engine.peopleNearProps, deltaSeconds);
}

// Moves the props forward and puts them into the frame.
void UpdateProps(Engine& engine, float deltaSeconds, FrameInput& frame) {
    if (engine.props.Count() == 0) {
        return;
    }
    GivePeopleToProps(engine, deltaSeconds);
    engine.props.Update(deltaSeconds);
    frame.bodyBoxes = engine.props.Boxes();
    frame.bodySpheres = engine.props.Spheres();
}

// Poses the characters for this moment and puts them into the frame.
void UpdateCharacters(Engine& engine, float worldSeconds, FrameInput& frame) {
    engine.characters.Update(worldSeconds, frame);
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

// Polls window events. Returns false once the window should close. Escape closes it unless `escapeCloses` is false, which is
// the case while the editor is open, where Escape cancels what is being done and must not end the program.
bool KeepRunning(GLFWwindow* window, bool escapeCloses = true) {
    glfwPollEvents();
    if (escapeCloses && glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
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
                      engine.PeopleCount(), engine.visibleNpcs.size(), crowdMs);
        glfwSetWindowTitle(window, title);
    }

    GLFWwindow* window;
    std::string deviceName;
    Clock::time_point lastUpdate;
    int frames = 0;
    double crowdMsTotal = 0.0;
};

FlyCamera MakeStartCamera(const Options& options, const Engine& engine) {
    // Without a camera of its own, a run with characters starts in front of them, because from far above they are a few pixels.
    const std::optional<CameraPlacement>& placement = options.camera ? options.camera : engine.characters.ViewPoint();
    if (placement) {
        return FlyCamera(placement->position, glm::radians(placement->yawDegrees), glm::radians(placement->pitchDegrees));
    }
    if (engine.streamed) {
        return FlyCamera(glm::vec3(0.0f, 60.0f, 0.0f), 0.0f, -0.25f);  // above the origin, looking along +Z
    }
    const CityLayout& city = engine.city;
    // Above the south-west of the city, looking towards its centre.
    const glm::vec3 start(city.coreBounds.min.x * 0.55f, 70.0f, city.coreBounds.min.y * 0.55f);
    return FlyCamera(start, std::atan2(-start.x, -start.z), -0.32f);
}

// What the engine host may reach in the running program.
engine_host::WorldAccess MakeWorldAccess(Engine& engine, FlyCamera& camera, graph_engine::SunSettings& sun, const Options& options) {
    engine_host::WorldAccess access;
    access.camera = &camera;
    access.sun = &sun;
    access.city = &engine.city;
    access.crowd = &engine.crowd;
    access.props = &engine.props;
    access.river = &engine.river;
    access.renderer = engine.renderer;
    access.worldSeed = options.seed;
    return access;
}

int RunInteractive(Engine& engine, const Options& options) {
    FlyCamera camera = MakeStartCamera(options, engine);
    graph_engine::SunSettings sun = DefaultSun(options);
    TitleBar title(engine.window, engine.renderer->GetDeviceName());
    engine_host::LiveEngineHost host(MakeWorldAccess(engine, camera, sun, options));
    editor_ui::EditorSession editor(*engine.renderer, engine.window, options, &host);
    engine.previousCameraPosition = camera.Position();

    const Clock::time_point startTime = Clock::now();
    Clock::time_point previousFrame = startTime;
    while (KeepRunning(engine.window, !editor.IsVisible())) {
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

        host.BeginFrame(deltaSeconds);
        editor.BeginFrame(deltaSeconds);
        // The camera and the sun keys leave the pointer and the keys to the editor while it is using them. The camera still
        // sees every frame, so that a look that began before the editor took the pointer ends when the button is let go.
        camera.Update(engine.window, deltaSeconds, !editor.CapturesMouse(), !editor.CapturesKeyboard());
        if (!editor.CapturesKeyboard() && glfwGetKey(engine.window, GLFW_KEY_Z) == GLFW_PRESS) {
            sun.elevationDegrees -= kSunTurnSpeed * deltaSeconds;
        }
        if (!editor.CapturesKeyboard() && glfwGetKey(engine.window, GLFW_KEY_X) == GLFW_PRESS) {
            sun.elevationDegrees += kSunTurnSpeed * deltaSeconds;
        }
        sun.elevationDegrees = std::clamp(sun.elevationDegrees, -5.0f, 85.0f);
        UpdateStreaming(engine, camera, deltaSeconds);

        const float worldSeconds = options.worldTimeSeconds.value_or(0.0f) +
                                   static_cast<float>(MillisecondsBetween(startTime, now) / 1000.0);
        FrameInput frame = MakeFrameInput(camera, aspect, sun, worldSeconds);
        const double crowdMs = UpdateCrowd(engine, deltaSeconds, frame, &host);
        UpdateProps(engine, deltaSeconds, frame);
        UpdateCharacters(engine, worldSeconds, frame);
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
    FlyCamera camera = MakeStartCamera(options, engine);
    const graph_engine::SunSettings sun = DefaultSun(options);
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
        FrameInput frame = MakeFrameInput(camera, aspect, sun, worldSeconds);
        const double crowdMs = UpdateCrowd(engine, kFixedTimeStep, frame);
        UpdateProps(engine, kFixedTimeStep, frame);
        UpdateCharacters(engine, worldSeconds, frame);
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

// What a streaming benchmark gathers besides frame times.
struct StreamRunTotals {
    std::vector<double> streamMs;  // main thread time of the streaming update, per frame
    double chunksDrawnByCamera = 0.0;
    double chunksDrawnBySun = 0.0;
    double drawCalls = 0.0;
    std::set<std::pair<int, int>> chunksFlownOver;
    size_t hitchFrames = 0;  // frames over the hitch limit
    int worstFrame = 0;
    double worstFrameMs = 0.0;

    // One frame, for finding out what the slowest ones were doing.
    struct FrameNote {
        int frame = 0;
        double frameMs = 0.0;
        double streamMs = 0.0;
        double crowdMs = 0.0;
        size_t chunksDelivered = 0;
        size_t chunksUnloaded = 0;
    };
    std::vector<FrameNote> notes;
};

// Prints the slowest frames with what streaming and the crowd cost in them and which chunks arrived or left.
void PrintSlowestFrames(std::vector<StreamRunTotals::FrameNote> notes, size_t count) {
    std::sort(notes.begin(), notes.end(), [](const auto& a, const auto& b) { return a.frameMs > b.frameMs; });
    std::printf("  slowest frames   frame: total ms (streaming update ms, crowd ms, chunks in / out)\n");
    for (size_t i = 0; i < std::min(count, notes.size()); ++i) {
        const auto& note = notes[i];
        std::printf("    %5d: %6.2f ms (%5.2f, %5.2f, +%zu / -%zu)\n", note.frame, note.frameMs, note.streamMs, note.crowdMs,
                    note.chunksDelivered, note.chunksUnloaded);
    }
}

// The samples of `all` from index `from` on, in milliseconds.
std::vector<double> SecondsToMillisecondsFrom(const std::vector<double>& all, size_t from) {
    std::vector<double> milliseconds;
    for (size_t i = from; i < all.size(); ++i) {
        milliseconds.push_back(all[i] * 1000.0);
    }
    return milliseconds;
}

void PrintStreamReport(const Engine& engine, const Options& options, const streaming::StreamingStats& before, const StreamRunTotals& totals,
                       const StreamFlightPath& path, size_t frames) {
    const streaming::StreamingStats after = engine.streamed->Stats();
    const FrameTimeSummary latency = SummarizeFrameTimes(SecondsToMillisecondsFrom(after.latencies, before.latencies.size()), 1e9);
    const FrameTimeSummary streamCost = SummarizeFrameTimes(totals.streamMs, 1e9);
    const RenderStats render = engine.renderer->GetStats();
    const double frameCount = static_cast<double>(std::max<size_t>(frames, 1));
    std::printf("Streaming: %.2f km flown, %zu chunks flown over, %zu people loaded at the end\n", path.DistanceFlown() / 1000.0,
                totals.chunksFlownOver.size(), engine.PeopleCount());
    std::printf("  chunks           %llu loaded, %llu unloaded, %llu requests dropped as stale, %llu failed; %zu resident at the end, %zu at most\n",
                static_cast<unsigned long long>(after.delivered - before.delivered),
                static_cast<unsigned long long>(after.unloaded - before.unloaded),
                static_cast<unsigned long long>(after.cancelled - before.cancelled),
                static_cast<unsigned long long>(after.failed - before.failed), after.residentChunks, after.residentChunksPeak);
    std::printf("  generation       request to resident p50 %.1f ms, p99 %.1f ms, worst %.1f ms\n", latency.medianMs, latency.p99Ms,
                latency.worstMs);
    std::printf("  memory           %.1f MB resident at most (chunk data); GPU instances %.1f MB of %.1f MB in use\n",
                static_cast<double>(after.residentBytesPeak) / (1024.0 * 1024.0),
                static_cast<double>(render.streamed.instancesUsed * sizeof(BoxInstance)) / (1024.0 * 1024.0),
                static_cast<double>(render.streamed.instancesCapacity * sizeof(BoxInstance)) / (1024.0 * 1024.0));
    std::printf("  upload           %.2f MB at most in one frame (manager %.2f MB), %.0f box draw calls a frame on average\n",
                static_cast<double>(render.streamed.uploadBytesMaxFrame) / (1024.0 * 1024.0),
                static_cast<double>(after.deliveredBytesMaxFrame) / (1024.0 * 1024.0), totals.drawCalls / frameCount);
    std::printf("  culling          %.1f chunks a frame drawn for the camera, %.1f for the sun, of %zu resident\n",
                totals.chunksDrawnByCamera / frameCount, totals.chunksDrawnBySun / frameCount, after.residentChunks);
    std::printf("  frame cost       streaming update on the main thread: average %.3f ms, p99 %.3f, worst %.3f ms\n",
                streamCost.averageMs, streamCost.p99Ms, streamCost.worstMs);
    std::printf("  hitches          %zu frames over %.0f ms (worst %.2f ms, frame %d)\n", totals.hitchFrames, options.hitchLimitMs,
                totals.worstFrameMs, totals.worstFrame);
    PrintSlowestFrames(totals.notes, 6);
}

// Flies the streaming flight path and reports. Fails (exit code 2) when p99 misses the target frame rate, when a frame takes
// longer than the hitch limit, or when the window closes early.
int RunStreamBenchmark(Engine& engine, const Options& options) {
    StreamFlightPath path(options.streamBenchmarkFrames);
    BenchmarkRecorder recorder(StreamFlightPath::LegNames());
    FlyCamera camera(glm::vec3(0.0f), 0.0f, 0.0f);
    const StreamFlightPath::Pose start = path.Start();
    camera.LookAt(start.position, start.target);
    engine.previousCameraPosition = start.position;
    engine.streamed->LoadAround(start.position);  // the world around the start is there before the clock starts
    const streaming::StreamingStats before = engine.streamed->Stats();
    TitleBar title(engine.window, engine.renderer->GetDeviceName());

    StreamRunTotals totals;
    const int totalFrames = kBenchmarkWarmupFrames + options.streamBenchmarkFrames;
    Clock::time_point previousFrame = Clock::now();
    int frameIndex = 0;
    for (; frameIndex < totalFrames && KeepRunning(engine.window); ++frameIndex) {
        const int measuredIndex = frameIndex - kBenchmarkWarmupFrames;
        if (measuredIndex >= 0) {
            const StreamFlightPath::Pose pose = path.Step(kFixedTimeStep);
            camera.LookAt(pose.position, pose.target);
        }
        const double streamMs = UpdateStreaming(engine, camera, kFixedTimeStep);
        const float aspect = std::max(WindowAspectRatio(engine.window), 0.01f);
        const float worldSeconds = static_cast<float>(frameIndex) * kFixedTimeStep;
        FrameInput frame = MakeFrameInput(camera, aspect, options.sunElevationDegrees.value_or(kDefaultSunElevation), worldSeconds);
        const double crowdMs = UpdateCrowd(engine, kFixedTimeStep, frame);
        engine.renderer->Render(frame);
        title.AddFrame(crowdMs, engine);

        const Clock::time_point now = Clock::now();
        const double frameMs = MillisecondsBetween(previousFrame, now);
        previousFrame = now;
        if (measuredIndex < 0) {
            continue;
        }
        const RenderStats render = engine.renderer->GetStats();
        recorder.Record({.cpuFrameMs = frameMs,
                         .crowdMs = crowdMs,
                         .peopleOnScreen = engine.visibleNpcs.size(),
                         .shotIndex = path.LegOfFrame(measuredIndex)},
                        render);
        totals.streamMs.push_back(streamMs);
        totals.chunksDrawnByCamera += render.streamed.chunksDrawnByCamera;
        totals.chunksDrawnBySun += render.streamed.chunksDrawnBySun;
        totals.drawCalls += render.streamed.drawCalls;
        const streaming::ChunkCoord here = world::ChunkAt({camera.Position().x, camera.Position().z});
        totals.chunksFlownOver.insert({here.x, here.z});
        if (frameMs > options.hitchLimitMs) {
            ++totals.hitchFrames;
        }
        if (frameMs > totals.worstFrameMs) {
            totals.worstFrameMs = frameMs;
            totals.worstFrame = measuredIndex;
        }
        totals.notes.push_back({measuredIndex, frameMs, streamMs, crowdMs, engine.streamed->DeliveredByLastUpdate(),
                                engine.streamed->UnloadedByLastUpdate()});
    }

    const bool passedTarget = recorder.PrintReport(engine.PeopleCount(), options.targetFps);
    PrintStreamReport(engine, options, before, totals, path, totals.streamMs.size());
    const bool noHitches = totals.hitchFrames == 0;
    std::printf("  hitch limit      %s\n", noHitches ? "PASS" : "FAIL");
    const bool completed = frameIndex == totalFrames;
    if (!completed) {
        std::cerr << "Benchmark stopped after " << frameIndex << " of " << totalFrames << " frames\n";
    }
    return passedTarget && noHitches && completed ? 0 : kBenchmarkFailedExitCode;
}

// Moves the crowd, the props and the river forward to `seconds` without drawing anything.
void PreRollWorld(Engine& engine, float seconds) {
    const bool coupled = engine.river.IsBuilt() && engine.props.Count() > 0;
    while (seconds > 0.0f) {
        const float step = std::min(seconds, kPreRollTimeStep);
        engine.crowd.SetMovableObstacles(engine.props.Obstacles());
        engine.crowd.Update(step);
        if (engine.props.Count() > 0) {
            GivePeopleToProps(engine, step);
        }
        if (coupled) {
            // The props and the water act on each other, so they run together, in pieces of a frame, and only for the
            // last minute: whatever fell in before that has settled, and the heap starts to fall when the minute starts.
            if (seconds <= kCoupledPreRollSeconds) {
                for (float left = step; left > 0.0f; left -= kFixedTimeStep) {
                    engine.props.Update(std::min(left, kFixedTimeStep));
                    engine.river.Update(std::min(left, kFixedTimeStep));
                }
            }
        } else {
            if (engine.props.Count() > 0) {
                // The physics catches up in pieces of a frame, so that none of the time is dropped.
                for (float left = step; left > 0.0f; left -= kFixedTimeStep) {
                    engine.props.Update(std::min(left, kFixedTimeStep));
                }
            }
            // The ripples of the river settle within half a minute, so only the last half minute of the time before a
            // screenshot is simulated: running the water for ten minutes would take a minute of the user's time.
            if (seconds <= kRiverPreRollSeconds) {
                engine.river.Update(step);  // does nothing unless the river is simulated
            }
        }
        seconds -= step;
    }
}

// Renders a few frames from a fixed camera at a fixed world time and saves the last one.
int RunScreenshot(Engine& engine, const Options& options) {
    // The camera and the sun are changed only by the graph of an editor that is there, through the engine host.
    FlyCamera camera = MakeStartCamera(options, engine);
    graph_engine::SunSettings sun = DefaultSun(options);
    const int frames = options.screenshotFrames;
    // The last frame shows the world at `--time`; the frames before it lead up to it.
    const float renderedSeconds = static_cast<float>(frames) * kFixedTimeStep;
    const float finalSeconds = options.worldTimeSeconds.value_or(renderedSeconds);
    const float firstFrameSeconds = std::max(finalSeconds - renderedSeconds, 0.0f);
    engine.crowd.SetFocusPoint(camera.Position());
    engine.previousCameraPosition = camera.Position();
    if (engine.streamed) {
        engine.streamed->LoadAround(camera.Position());  // nobody wants a screenshot of a world that is still arriving
    }
    // Only a screenshot that asks for the editor has one; every other screenshot is drawn exactly as before. A graph that cannot be
    // loaded ends the run before the long part: nobody is there to read a message, and a screenshot of the wrong graph would mislead.
    std::optional<engine_host::LiveEngineHost> host;
    std::optional<editor_ui::EditorSession> editor;
    if (options.editor) {
        host.emplace(MakeWorldAccess(engine, camera, sun, options));
        editor.emplace(*engine.renderer, engine.window, options, &*host);
        if (!editor->StartupError().empty()) {
            std::cerr << "Cannot take the screenshot: " << editor->StartupError() << "\n";
            return 1;
        }
    }
    if (!engine.streamed) {
        PreRollWorld(engine, firstFrameSeconds);
    }

    for (int frameIndex = 0; frameIndex < frames; ++frameIndex) {
        if (!KeepRunning(engine.window)) {
            return 1;
        }
        UpdateStreaming(engine, camera, kFixedTimeStep);
        const float aspect = std::max(WindowAspectRatio(engine.window), 0.01f);
        const float worldSeconds = firstFrameSeconds + static_cast<float>(frameIndex + 1) * kFixedTimeStep;
        // The graph runs first, so that what it does to the camera and the sun is in this frame.
        if (editor) {
            host->BeginFrame(kFixedTimeStep);
            editor->BeginFrame(kFixedTimeStep);
        }
        FrameInput frame = MakeFrameInput(camera, aspect, sun, worldSeconds);
        UpdateCrowd(engine, kFixedTimeStep, frame, host ? &*host : nullptr);
        UpdateProps(engine, kFixedTimeStep, frame);
        UpdateCharacters(engine, worldSeconds, frame);
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
        std::cout << ", river simulated around (" << engine.river.Centre().x << ", " << engine.river.Centre().y << ")";
    }
    if (engine.props.Count() > 0) {
        std::cout << ", " << engine.props.Count() << " props above the " << (engine.river.IsBuilt() ? "river" : "square") << " at ("
                  << engine.props.DropCentre().x << ", " << engine.props.DropCentre().y << ")";
    }
    std::cout << "\n";
    if (!options.IsBenchmark() && !options.IsScreenshot()) {
        std::cout << "Controls: hold the right mouse button to look, WASD to move, E/Q to go up/down,\n"
                  << "          Shift to go faster, Z/X to lower/raise the sun, Esc to quit\n"
                  << "          F1 opens the graph editor\n";
    }
}

// Generates the scanned materials from the world seed and hands them to the renderer. The generator is deterministic,
// so the same seed always draws the same walls and streets.
void UploadScanMaterials(IRenderModule& renderer, const Options& options) {
    const auto start = Clock::now();
    material::LibrarySettings settings;
    settings.seed = options.seed;
    settings.size = static_cast<int>(options.materialSize);
    const material::MaterialLibrary library = material::BuildMaterialLibrary(settings, &JobSystem::Shared());
    renderer.UploadMaterials(library);
    const double milliseconds = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    std::cout << "Materials: " << library.layers << " layers of " << library.size << " px made and uploaded in "
              << static_cast<int>(milliseconds) << " ms\n";
}

// The distance haze of the streamed world. It is thicker than the fixed city's, so that the far edge of the loaded chunks is
// well hidden in it: at the load radius the haze takes most of the colour of a surface.
constexpr float kStreamFogDensity = 0.0022f;

// Sets up the endless city instead of the fixed one. Props, the simulated river and the skinned characters of the showcase
// belong to the fixed city and are not made here.
void StartStreamedWorld(Engine& engine, const Options& options) {
    if (options.propCount > 0 || options.riverSimulation) {
        std::cerr << "--props and --river-sim are not streamed yet and are ignored with --stream\n";
    }
    if (options.scanMaterials) {
        UploadScanMaterials(*engine.renderer, options);
    }
    engine.renderer->SetFogDensity(kStreamFogDensity);
    engine.renderer->SetDistanceFade(options.streamLoadRadius);
    // Room for everyone up front, so the renderer never grows buffers mid-flight.
    engine.renderer->ReserveNpcCapacity(options.npcCount);
    engine.visibleNpcs.reserve(options.npcCount);
    engine.crowdCharacterLimit = options.charactersGiven ? options.characterCount : kStreamDefaultCharacters;
    if (engine.crowdCharacterLimit > 0) {
        engine.crowdCharacters.MakeBodies(*engine.renderer, options.seed);
    }
    streaming::StreamedWorldSettings settings;
    settings.seed = options.seed;
    settings.peopleTotal = options.npcCount;
    settings.loadRadius = options.streamLoadRadius;
    engine.streamed = std::make_unique<streaming::StreamedWorld>(*engine.renderer, settings);
    std::cout << "Streamed world (seed " << options.seed << "): chunks of " << world::kChunkSize << " m kept loaded within "
              << options.streamLoadRadius << " m of the camera, about " << options.npcCount << " people ("
              << engine.streamed->PeoplePerChunk() << " per chunk)\n";
    if (!options.IsBenchmark() && !options.IsStreamBenchmark() && !options.IsScreenshot()) {
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

    if (options.stream) {
        StartStreamedWorld(engine, options);
        if (options.IsStreamBenchmark()) {
            return RunStreamBenchmark(engine, options);
        }
        if (options.IsScreenshot()) {
            return RunScreenshot(engine, options);
        }
        return RunInteractive(engine, options);
    }

    // Generate the world.
    CitySettings citySettings;
    citySettings.seed = options.seed;
    if (options.satelliteCount) {
        citySettings.satelliteCount = *options.satelliteCount;
    }
    citySettings.terraces = options.terraces;
    engine.city = GenerateCity(citySettings);
    engine.renderer->UploadStaticScene(engine.city.boxes);
    if (options.scanMaterials) {
        UploadScanMaterials(*engine.renderer, options);
    }
    engine.crowd.Spawn(engine.city, {.agentCount = options.npcCount, .seed = options.seed * 7919u + 1u});
    // Reserve room for everyone up front, so the renderer never grows buffers mid-flight.
    engine.renderer->ReserveNpcCapacity(engine.crowd.AgentCount());
    engine.visibleNpcs.reserve(engine.crowd.AgentCount());
    // The river nearest the middle of the city, if it is to be simulated, and then the props are dropped over it.
    fluid::RiverWaterSettings riverSettings;
    riverSettings.rainPerSecond = static_cast<float>(options.rainPerSecond);
    riverSettings.seed = options.seed;
    if (options.riverSimulation && !engine.river.Build(engine.city, glm::vec2(0.0f), riverSettings)) {
        std::cerr << "The city has no water to simulate\n";
    }
    if (options.propCount > 0) {
        physics::PropSettings props{.count = options.propCount, .seed = options.seed};
        if (engine.river.IsBuilt()) {
            props.dropAt = engine.river.Centre();
            props.riverBed = engine.river.RestLevel() - riverSettings.depth;
        }
        engine.props.Spawn(engine.city, props);
        if (engine.river.IsBuilt()) {
            engine.props.SetWater(&engine.river);
        }
    }
    engine.characters.Spawn(*engine.renderer, engine.city, options.characterCount, options.seed);
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
