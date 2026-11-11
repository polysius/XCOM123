// XCOM123 entry point: opens a window, starts the renderer module, generates a city and its
// crowd, then runs the frame loop.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>

#include "core/FlyCamera.h"
#include "core/Module_Interface.h"
#include "render/IRenderModule.h"
#include "scene/Frustum.h"
#include "world/CityGenerator.h"
#include "world/Crowd.h"

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
    uint32_t seed = 1789;
    uint32_t npcCount = 10000;
    bool vsync = true;
    int benchmarkFrames = 0;  // above 0: render this many frames, print timings, then quit
    bool showHelp = false;
};

constexpr int kBenchmarkWarmupFrames = 30;
constexpr float kSunAzimuthDegrees = 230.0f;
constexpr float kSunTurnSpeed = 10.0f;  // degrees per second while Z or X is held

void PrintUsage() {
    std::cout << "Usage: XCOM123 [--seed N] [--npcs N] [--novsync] [--benchmark FRAMES]\n"
                 "  --seed N            city and crowd seed (default 1789)\n"
                 "  --npcs N            number of people (default 10000)\n"
                 "  --novsync           do not wait for the display's refresh\n"
                 "  --benchmark FRAMES  render FRAMES frames without vsync, print timings, then quit\n";
}

// Throws std::invalid_argument (or std::out_of_range) on input it cannot use.
Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const bool hasValue = i + 1 < argc;
        if (argument == "--seed" && hasValue) {
            options.seed = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (argument == "--npcs" && hasValue) {
            options.npcCount = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (argument == "--benchmark" && hasValue) {
            options.benchmarkFrames = std::stoi(argv[++i]);
            options.vsync = false;  // measure how fast the engine is, not the display's refresh rate
        } else if (argument == "--novsync") {
            options.vsync = false;
        } else if (argument == "--help") {
            options.showHelp = true;
        } else {
            throw std::invalid_argument("Unknown argument: " + argument);
        }
    }
    return options;
}

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

double Average(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

void PrintBenchmark(std::vector<double> frameMs, const std::vector<double>& crowdMs, size_t people, size_t onScreen) {
    std::sort(frameMs.begin(), frameMs.end());
    const double averageFrame = Average(frameMs);
    const double slowFrame = frameMs[std::min(frameMs.size() - 1, frameMs.size() * 99 / 100)];
    std::printf("Benchmark: %zu frames, %zu people (%zu on screen)\n", frameMs.size(), people, onScreen);
    std::printf("  frame time  %.2f ms average (%.0f fps), %.2f ms at the 99th percentile\n",
                averageFrame, 1000.0 / averageFrame, slowFrame);
    std::printf("  crowd       %.3f ms average for simulation and culling\n", Average(crowdMs));
}

// Runs the engine until the window closes (or the benchmark ends). Returns the process exit code.
int RunEngine(const Options& options, GLFWwindow* window) {
    EngineContext engine;
    engine.window = window;
    engine.vsync = options.vsync;

    ModuleRegistry& modules = ModuleRegistry::Instance();
    if (!modules.CreateModule("Vulkan", engine)) {
        return 1;
    }
    IRenderModule* renderer = modules.GetModule<IRenderModule>("Vulkan");
    glfwSetWindowUserPointer(window, renderer);
    glfwSetFramebufferSizeCallback(window, OnFramebufferResized);
    modules.ListModules();

    // Generate the world.
    CitySettings citySettings;
    citySettings.seed = options.seed;
    const CityLayout city = GenerateCity(citySettings);
    renderer->UploadStaticScene(city.boxes);

    Crowd crowd;
    crowd.Spawn(city, {.agentCount = options.npcCount, .seed = options.seed * 7919u + 1u});

    std::cout << "City (seed " << options.seed << "): " << city.boxes.size() << " boxes, "
              << city.streets.edges.size() << " street segments, " << city.plazas.size() << " squares, "
              << crowd.AgentCount() << " people\n"
              << "Controls: hold the right mouse button to look, WASD to move, E/Q to go up/down,\n"
              << "          Shift to go faster, Z/X to lower/raise the sun, Esc to quit\n";

    // Start above the south-west of the city, looking towards its centre.
    const glm::vec3 start(city.bounds.min.x * 0.55f, 70.0f, city.bounds.min.y * 0.55f);
    FlyCamera camera(start, std::atan2(-start.x, -start.z), -0.32f);
    float sunElevation = 24.0f;

    std::vector<NpcInstance> visibleNpcs;
    visibleNpcs.reserve(crowd.AgentCount());
    std::vector<double> benchmarkFrameMs;
    std::vector<double> benchmarkCrowdMs;

    const Clock::time_point startTime = Clock::now();
    Clock::time_point previousFrame = startTime;
    Clock::time_point lastTitleUpdate = startTime;
    int framesSinceTitle = 0;
    double crowdMsSinceTitle = 0.0;
    int frameNumber = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        if (width == 0 || height == 0) {
            glfwWaitEvents();  // minimised: sleep until something happens
            previousFrame = Clock::now();
            continue;
        }

        const Clock::time_point now = Clock::now();
        const double frameMs = MillisecondsBetween(previousFrame, now);
        previousFrame = now;
        // Cap the time step, so a long stall (such as dragging the window) does not teleport everyone.
        const float deltaSeconds = static_cast<float>(std::min(frameMs / 1000.0, 0.1));

        camera.Update(window, deltaSeconds);
        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
            sunElevation -= kSunTurnSpeed * deltaSeconds;
        }
        if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
            sunElevation += kSunTurnSpeed * deltaSeconds;
        }
        sunElevation = std::clamp(sunElevation, -5.0f, 85.0f);

        FrameInput frame;
        frame.view = camera.ViewMatrix();
        frame.projection = camera.ProjectionMatrix(static_cast<float>(width) / static_cast<float>(height));
        frame.cameraPosition = camera.Position();
        frame.sunDirection = SunDirection(sunElevation, kSunAzimuthDegrees);
        frame.timeSeconds = static_cast<float>(MillisecondsBetween(startTime, now) / 1000.0);

        const Clock::time_point crowdStart = Clock::now();
        crowd.Update(deltaSeconds);
        crowd.CollectVisible(Frustum(frame.projection * frame.view), visibleNpcs);
        const double crowdMs = MillisecondsBetween(crowdStart, Clock::now());
        frame.npcs = visibleNpcs;

        renderer->Render(frame);

        ++frameNumber;
        ++framesSinceTitle;
        crowdMsSinceTitle += crowdMs;
        if (options.benchmarkFrames > 0 && frameNumber > kBenchmarkWarmupFrames) {
            benchmarkFrameMs.push_back(frameMs);
            benchmarkCrowdMs.push_back(crowdMs);
            if (static_cast<int>(benchmarkFrameMs.size()) >= options.benchmarkFrames) {
                break;
            }
        }

        const double msSinceTitle = MillisecondsBetween(lastTitleUpdate, now);
        if (msSinceTitle >= 500.0) {
            const double averageMs = msSinceTitle / framesSinceTitle;
            char title[256];
            std::snprintf(title, sizeof(title), "XCOM123 | %s | %.2f ms (%.0f fps) | %zu people, %zu on screen | crowd %.2f ms",
                          renderer->GetDeviceName().c_str(), averageMs, 1000.0 / averageMs, crowd.AgentCount(),
                          visibleNpcs.size(), crowdMsSinceTitle / framesSinceTitle);
            glfwSetWindowTitle(window, title);
            lastTitleUpdate = now;
            framesSinceTitle = 0;
            crowdMsSinceTitle = 0.0;
        }
    }

    if (!benchmarkFrameMs.empty()) {
        PrintBenchmark(benchmarkFrameMs, benchmarkCrowdMs, crowd.AgentCount(), visibleNpcs.size());
    }
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    Options options;
    try {
        options = ParseOptions(argc, argv);
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
