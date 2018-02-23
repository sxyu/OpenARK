#pragma once
#include "Detector.h"
#include "PlaneDetector.h"
#include "Hand.h"

namespace ark {
    /** Hand detector class supporting the detection of multiple hands within a depth projection image (xyz map).
     * @see PlaneDetector
     */
    class HandDetector : public Detector {
    public:

        /**
         * Construct a new hand detector instance.
         * @param elim_planes if true, removes major planes from the image before proceeding with hand detection.
         *                    A new instance of PlaneDetector will be constructed to remove planes.
         */
        explicit HandDetector(bool elim_planes = true, DetectionParams::Ptr params = nullptr);

        /**
         * Construct a new hand detector instance, using the provided plane detector to remove planes in each frame.
         * If planes and hands both need to be detected for your application,
         * this reduces redundancy as planes will only be detected once per frame.
         * @param plane_detector pointer to the plane detector instance used for finding planes.
         */
        explicit HandDetector(PlaneDetector::Ptr plane_detector, DetectionParams::Ptr params = nullptr);

        /**
         * Obtain a list of hands in the current frame from this detector.
         */
        std::vector<Hand::Ptr> getHands() const;

        /** Shared pointer to HandDetector instance */
        typedef std::shared_ptr<HandDetector> Ptr;

    protected:
        /** Implementation of hand detection algorithm */
        void detect(cv::Mat & image) override;

    private:
        /** the plane detector instance */
        PlaneDetector::Ptr planeDetector;

        /** stores currently detected hands */
        std::vector<Hand::Ptr> hands;
    };
}