#include "Detector.h"
#include "DetectionParams.h"

namespace ark {
    Detector::Detector(DetectionParams::Ptr params)
        : params(params ? params : DetectionParams::DEFAULT) {
        callback = std::bind(&Detector::callbackHelper, this, std::placeholders::_1);
    } 

    void Detector::update(const MultiCameraFrame & frame)
    {
        detect(frame);
        lastCamera = nullptr;
        onSameFrame = false;
    }

    void Detector::update(DepthCamera & camera)
    {
        // stop if the camera is still on the same frame as before
        if (onSameFrame && lastCamera == &camera) return;
        const MultiCameraFrame::Ptr frame = camera.getFrame();
        if (frame == nullptr) return;
        detect(*frame);

        if (lastCamera != &camera) {
            if (lastCamera) lastCamera->removeUpdateCallback(lastUpdateCallbackID);
            lastUpdateCallbackID = camera.addUpdateCallback(callback);
        }

        lastCamera = &camera;
    }

    void Detector::setParams(const DetectionParams::Ptr params)
    {
        this->params = params;
    }

    void Detector::callbackHelper(DepthCamera & camera)
    {
        onSameFrame = false;
    }
}
