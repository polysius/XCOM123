#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include "core/Module_Interface.h"
#include "scene/SceneTypes.h"

// GPU time spent in one render pass.
struct GpuPassTiming {
    std::string_view name;
    double latestMs = 0.0;    // the most recent measured frame
    double smoothedMs = 0.0;  // averaged over roughly the last ten frames
};

// What the renderer measured and decided, for the window title, benchmarks and tuning.
// GPU times arrive one or two frames late, because they are read back without waiting.
struct RenderStats {
    bool gpuTimingAvailable = false;
    std::span<const GpuPassTiming> passes;  // valid until the next Render call
    double gpuFrameMs = 0.0;
    double smoothedGpuFrameMs = 0.0;
    uint64_t gpuMeasuredFrames = 0;  // grows by one each time new GPU times arrive
    float renderScale = 1.0f;        // fraction of the window size the scene is drawn at
    uint32_t renderWidth = 0;
    uint32_t renderHeight = 0;
};

// How the renderer picks the resolution the scene is drawn at.
struct ResolutionSettings {
    bool dynamic = true;       // adjust the render scale to keep GPU time under the budget
    float fixedScale = 1.0f;   // the scale used when `dynamic` is false
    float gpuBudgetMs = 14.0f; // GPU time per frame the dynamic scale aims to stay under
};

// Specific Module Interface: what the rest of the engine may ask of a renderer.
class IRenderModule : public IModule {
public:
    // Called when the window's drawable area changes size.
    virtual void SetResolution(int width, int height) = 0;
    virtual void SetResolutionSettings(const ResolutionSettings& settings) = 0;
    // Static geometry is uploaded once; the renderer keeps its own copy on the GPU.
    virtual void UploadStaticScene(std::span<const BoxInstance> boxes) = 0;
    // Makes room for this many visible people up front, so the first crowded frames do not
    // have to grow GPU buffers.
    virtual void ReserveNpcCapacity(size_t npcCount) = 0;
    // Draws one frame. The data in `frame` only has to stay valid during the call.
    virtual void Render(const FrameInput& frame) = 0;
    // Draws one frame, waits for it to finish and saves what it shows as a PNG file.
    // Returns false if the frame could not be drawn (for example a minimised window) or saved.
    virtual bool RenderAndCapture(const FrameInput& frame, const std::string& pngPath) = 0;
    virtual RenderStats GetStats() const = 0;
    virtual std::string GetDeviceName() const = 0;
};
