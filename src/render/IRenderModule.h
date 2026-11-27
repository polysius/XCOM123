#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>

#include "core/Module_Interface.h"
#include "material/MaterialLibrary.h"
#include "render/IFrameOverlay.h"
#include "render/SkinningMath.h"
#include "scene/Aabb.h"
#include "scene/SceneTypes.h"

// GPU time spent in one render pass.
struct GpuPassTiming {
    std::string_view name;
    double latestMs = 0.0;    // the most recent measured frame
    double smoothedMs = 0.0;  // averaged over roughly the last ten frames
};

// A chunk of the streamed world that was handed to the renderer (see IRenderModule::AddStreamedChunk).
struct StreamedChunkHandle {
    static constexpr uint32_t kInvalid = 0xFFFFFFFFu;
    uint32_t index = kInvalid;

    bool IsValid() const { return index != kInvalid; }
};

// What the streamed world costs the renderer: how many chunks are in the arena and drawn, and what was copied to the GPU.
struct StreamedGeometryStats {
    uint32_t chunksResident = 0;        // uploaded and drawable
    uint32_t chunksPending = 0;         // added, waiting for their copy
    uint32_t chunksDrawnByCamera = 0;   // inside the camera's view this frame
    uint32_t chunksDrawnBySun = 0;      // inside the sun's view this frame
    uint32_t drawCalls = 0;             // box draw calls of the camera pass and the shadow pass together
    size_t uploadBytesThisFrame = 0;
    size_t uploadBytesMaxFrame = 0;
    size_t instancesUsed = 0;
    size_t instancesCapacity = 0;
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
    StreamedGeometryStats streamed;  // all zero unless chunks were added
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
    // The streamed world. A chunk is added without waiting for the GPU and is drawn from the frame after its boxes were
    // copied (normally the next one); `owner` keeps the memory `boxes` points into alive until then. Each frame the renderer
    // draws the chunks that are in view of the camera and, for shadows, of the sun. Removing a chunk stops drawing it at once
    // and gives its room back when the frames that may still draw it have finished. Returns an invalid handle for an empty chunk.
    virtual StreamedChunkHandle AddStreamedChunk(std::shared_ptr<const void> owner, std::span<const BoxInstance> boxes,
                                                 const Aabb& bounds) = 0;
    virtual void RemoveStreamedChunk(StreamedChunkHandle chunk) = 0;
    // How thick the distance haze is. The streamed world sets it thicker, so that chunks fade in out of the haze at the
    // edge of the load radius instead of appearing.
    virtual void SetFogDensity(float density) = 0;
    // Distance from the camera, along the ground, at which everything has faded into the haze completely. Zero, the default,
    // means no fade. The streamed world sets its load radius, so that the edge of the loaded chunks is never seen.
    virtual void SetDistanceFade(float radius) = 0;
    // Gives the renderer the generated materials to draw surfaces with. Until this is called, surfaces are drawn
    // with the city colours and procedural detail only.
    virtual void UploadMaterials(const material::MaterialLibrary& library) = 0;
    // Makes room for this many visible people up front, so the first crowded frames do not
    // have to grow GPU buffers.
    virtual void ReserveNpcCapacity(size_t npcCount) = 0;
    // Uploads the triangles of a skinned character once and returns a handle to draw it with. Each frame, the characters to draw
    // are given as FrameInput::characters, each with its own palette of skinning matrices. Returns an invalid handle when the mesh
    // is empty or has more joints than a palette can name.
    virtual CharacterHandle UploadCharacterMesh(const skinning::MeshData& mesh) = 0;
    // Says which mesh to draw a body with when it is far from the camera, and (one level coarser again) in shadows. Both come from
    // UploadCharacterMesh and must have palettes of the same size. Chain them for more levels: full, coarser, coarsest.
    // Returns false when the two cannot be linked.
    virtual bool SetCharacterLowerDetail(CharacterHandle body, CharacterHandle lowerDetail) = 0;
    // Draws one frame. The data in `frame` only has to stay valid during the call.
    virtual void Render(const FrameInput& frame) = 0;
    // Draws one frame, waits for it to finish and saves what it shows as a PNG file.
    // Returns false if the frame could not be drawn (for example a minimised window) or saved.
    virtual bool RenderAndCapture(const FrameInput& frame, const std::string& pngPath) = 0;
    virtual RenderStats GetStats() const = 0;
    virtual std::string GetDeviceName() const = 0;
    // Puts something to draw over every finished frame, such as an editor, or takes it away again with null. The renderer
    // does not own it: the caller takes it away before it is destroyed. Without one, frames are drawn as they always were.
    virtual void SetOverlay(IFrameOverlay* overlay) = 0;
};
