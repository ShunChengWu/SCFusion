// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../../Utils/ITMMath.h"

#define DEBUG_CORRESPONDENCES 0


    /**
     * \brief An instance of this struct represents a surfel that does not contain colour information.
     */
    typedef struct ITMSurfel_grey {
        static const CONSTPTR(bool) hasColourInformation = false;
        static const CONSTPTR(bool) hasLabelInformation = false;

        // Note: The ordering of the variables here matters because it affects padding - do not reorder without prior thought.

        /** The surface normal at the surfel. */
        Vector3f normal;

        /** The position of the surfel. */
        Vector3f position;

        /** The confidence counter for the surfel. */
        float confidence;

        /** The radius of the surfel. */
        float radius;

        /** A timestamp for the surfel, recording the last frame in which it was updated. */
        int timestamp;

#if DEBUG_CORRESPONDENCES
        /** The new position of the surfel (prior to fusing). */
        Vector3f newPosition;

        /** The old position of the surfel (prior to fusing). */
        Vector3f oldPosition;
#endif
    } ITMSurfel_grey;

    /**
     * \brief An instance of this struct represents a surfel that contains colour information.
     */
    typedef struct ITMSurfel_rgb {
        static const CONSTPTR(bool) hasColourInformation = true;
        static const CONSTPTR(bool) hasLabelInformation = false;

        // Note: The ordering of the variables here matters because it affects padding - do not reorder without prior thought.

        /** The RGB colour of the surfel. */
        Vector3u colour;

        /** The surface normal at the surfel. */
        Vector3f normal;

        /** The position of the surfel. */
        Vector3f position;

        /** The confidence counter for the surfel. */
        float confidence;

        /** The radius of the surfel. */
        float radius;

        /** A timestamp for the surfel, recording the last frame in which it was updated. */
        int timestamp;

#if DEBUG_CORRESPONDENCES
        /** The new position of the surfel (prior to fusing). */
        Vector3f newPosition;

        /** The old position of the surfel (prior to fusing). */
        Vector3f oldPosition;
#endif
    } ITMSurfel_rgb;

    typedef struct Surfel_rgb_1label {
        static const CONSTPTR(bool) hasColourInformation = true;
        static const CONSTPTR(bool) hasLabelInformation = true;

        // Note: The ordering of the variables here matters because it affects padding - do not reorder without prior thought.

        /** The RGB colour of the surfel. */
        Vector3u colour;

        /** The surface normal at the surfel. */
        Vector3f normal;

        /** The position of the surfel. */
        Vector3f position;

        /** The confidence counter for the surfel. */
        float confidence;

        /** The radius of the surfel. */
        float radius;

        /** A timestamp for the surfel, recording the last frame in which it was updated. */
        int timestamp;

        ushort label;
        float w_label;

//        Surfel_rgb_1label(){
//            label = w_label = 0;
//        }

#if DEBUG_CORRESPONDENCES
        /** The new position of the surfel (prior to fusing). */
    Vector3f newPosition;

    /** The old position of the surfel (prior to fusing). */
    Vector3f oldPosition;
#endif
    } Surfel_rgb_1label;
