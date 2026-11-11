#include "core/CommandLine.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
// Reads the value that follows an option, or throws when the command line ends too early.
class ArgumentReader {
public:
    ArgumentReader(int argc, char** argv) : count(argc), values(argv) {}

    bool HasMore() const { return next < count; }
    std::string Next() { return values[next++]; }

    std::string ValueFor(const std::string& option) {
        if (!HasMore()) {
            throw std::invalid_argument(option + " needs a value");
        }
        return Next();
    }

private:
    int count;
    char** values;
    int next = 1;  // argv[0] is the program itself
};

float ParseFloatInRange(const std::string& option, const std::string& text, float minimum, float maximum) {
    const float value = std::stof(text);
    // Written so that NaN, which compares false with everything, is rejected too.
    if (!(value >= minimum && value <= maximum)) {
        std::ostringstream message;
        message << option << " must be between " << minimum << " and " << maximum;
        throw std::invalid_argument(message.str());
    }
    return value;
}

// std::stoul accepts "-1" and wraps it around, so a leading minus sign is rejected up front.
uint32_t ParseUnsigned(const std::string& option, const std::string& text) {
    const size_t firstCharacter = text.find_first_not_of(" \t");
    if (firstCharacter != std::string::npos && text[firstCharacter] == '-') {
        throw std::invalid_argument(option + " must not be negative");
    }
    const unsigned long value = std::stoul(text);
    if (value > std::numeric_limits<uint32_t>::max()) {
        throw std::invalid_argument(option + " is too large");
    }
    return static_cast<uint32_t>(value);
}

int ParsePositiveInt(const std::string& option, const std::string& text) {
    const int value = std::stoi(text);
    if (value <= 0) {
        throw std::invalid_argument(option + " must be above zero");
    }
    return value;
}

// "x,y,z,yawDeg,pitchDeg"
CameraPlacement ParseCameraPlacement(const std::string& text) {
    std::vector<float> numbers;
    std::stringstream stream(text);
    std::string part;
    while (std::getline(stream, part, ',')) {
        numbers.push_back(std::stof(part));
        if (!std::isfinite(numbers.back())) {
            throw std::invalid_argument("--camera values must be finite numbers");
        }
    }
    if (numbers.size() != 5) {
        throw std::invalid_argument("--camera expects x,y,z,yawDeg,pitchDeg");
    }
    return {.position = {numbers[0], numbers[1], numbers[2]}, .yawDegrees = numbers[3], .pitchDegrees = numbers[4]};
}

void CheckCombinations(const Options& options) {
    if (options.IsBenchmark() && options.IsScreenshot()) {
        throw std::invalid_argument("--benchmark and --screenshot cannot be combined");
    }
    if (options.IsBenchmark() && options.camera) {
        throw std::invalid_argument("--camera cannot be combined with --benchmark, which flies its own path");
    }
    if (options.targetFps && !options.IsBenchmark()) {
        throw std::invalid_argument("--target-fps only applies to --benchmark");
    }
}
}  // namespace

Options ParseCommandLine(int argc, char** argv) {
    Options options;
    ArgumentReader arguments(argc, argv);
    while (arguments.HasMore()) {
        const std::string option = arguments.Next();
        if (option == "--seed") {
            options.seed = ParseUnsigned(option, arguments.ValueFor(option));
        } else if (option == "--npcs") {
            options.npcCount = ParseUnsigned(option, arguments.ValueFor(option));
        } else if (option == "--novsync") {
            options.vsync = false;
        } else if (option == "--benchmark") {
            options.benchmarkFrames = ParsePositiveInt(option, arguments.ValueFor(option));
        } else if (option == "--target-fps") {
            options.targetFps = ParseFloatInRange(option, arguments.ValueFor(option), 1.0f, 1000.0f);
        } else if (option == "--screenshot") {
            options.screenshotPath = arguments.ValueFor(option);
        } else if (option == "--screenshot-frames") {
            options.screenshotFrames = ParsePositiveInt(option, arguments.ValueFor(option));
        } else if (option == "--camera") {
            options.camera = ParseCameraPlacement(arguments.ValueFor(option));
        } else if (option == "--time") {
            options.worldTimeSeconds = ParseFloatInRange(option, arguments.ValueFor(option), 0.0f, 3600.0f);
        } else if (option == "--sun") {
            options.sunElevationDegrees = ParseFloatInRange(option, arguments.ValueFor(option), -5.0f, 89.0f);
        } else if (option == "--render-scale") {
            options.renderScale = ParseFloatInRange(option, arguments.ValueFor(option), 0.5f, 1.0f);
        } else if (option == "--no-dynres") {
            options.dynamicResolution = false;
        } else if (option == "--gpu-budget") {
            options.gpuBudgetMs = ParseFloatInRange(option, arguments.ValueFor(option), 1.0f, 100.0f);
        } else if (option == "--help") {
            options.showHelp = true;
        } else {
            throw std::invalid_argument("Unknown argument: " + option);
        }
    }

    // Benchmarks and screenshots measure or show the engine itself, not the display's refresh rate.
    if (options.IsBenchmark() || options.IsScreenshot()) {
        options.vsync = false;
    }
    CheckCombinations(options);
    return options;
}

void PrintUsage() {
    std::cout <<
        "Usage: XCOM123 [options]\n"
        "World:\n"
        "  --seed N                 city and crowd seed (default 1789)\n"
        "  --npcs N                 number of people (default 10000)\n"
        "  --sun DEGREES            sun elevation above the horizon (default 24)\n"
        "  --time SECONDS           world clock of the screenshot (0 to 3600); the crowd is simulated\n"
        "                           up to it\n"
        "  --camera x,y,z,yaw,pitch start camera, angles in degrees (yaw 0 looks along +Z)\n"
        "Display:\n"
        "  --novsync                do not wait for the display's refresh\n"
        "  --render-scale X         draw the scene at a fixed X (0.5 to 1.0) of the window size\n"
        "  --no-dynres              keep the render scale fixed (1.0 unless --render-scale is given)\n"
        "  --gpu-budget MS          GPU frame time the dynamic resolution aims to stay under (default 14)\n"
        "Measuring:\n"
        "  --benchmark FRAMES       fly a fixed camera path for FRAMES frames without vsync, then report\n"
        "                           (exit code 2 if the window closes before all frames are measured)\n"
        "  --target-fps N           with --benchmark: fail (exit code 2) when p99 frame time exceeds 1000/N ms\n"
        "  --screenshot FILE.png    render a few frames without vsync, save the last one, then quit\n"
        "  --screenshot-frames N    frames to render before the screenshot (default 60); screenshots use\n"
        "                           render scale 1.0 unless --render-scale is given\n";
}
