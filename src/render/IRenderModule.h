#pragma once

#include <span>
#include <string>

#include "core/Module_Interface.h"
#include "scene/SceneTypes.h"

// Specific Module Interface: what the rest of the engine may ask of a renderer.
class IRenderModule : public IModule {
public:
	// Called when the window's drawable area changes size.
	virtual void SetResolution(int width, int height) = 0;
	// Static geometry is uploaded once; the renderer keeps its own copy on the GPU.
	virtual void UploadStaticScene(std::span<const BoxInstance> boxes) = 0;
	// Draws one frame. The data in `frame` only has to stay valid during the call.
	virtual void Render(const FrameInput& frame) = 0;
	virtual std::string GetDeviceName() const = 0;
};
