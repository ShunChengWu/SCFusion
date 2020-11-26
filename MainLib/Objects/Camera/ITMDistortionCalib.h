// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"
#include <vector>

namespace ITMLib
{
	/** \brief
	    Represents the calibration information to compute a depth
	    image from the disparity image typically received from a
	    Kinect.
	*/
	class ITMDistortionCalib
	{
		//#################### ENUMERATIONS ####################
	public:


		//#################### PRIVATE VARIABLES ####################
	private:
		size_t num;

		/** These are the actual parameters. */
        Vector8f params;

		//#################### CONSTRUCTORS ####################
	public:
		ITMDistortionCalib(void)
		{
			SetStandard();
		}

		//#################### PUBLIC MEMBER FUNCTIONS ####################
	public:
		const Vector8f& GetParams() const
		{
			return params;
		}
        // Get the number of distortion calibration parameters
		size_t GetNum() const
		{
			return num;
		}

		void SetFrom(std::vector<float> val)
		{
			if(val.size() > 8) {
				throw "size cannot excess 8\n";
			}
			for(size_t i=0; i<val.size(); ++i){
				params[i]=val[i];
			}
		}

		/** Setup from given arguments. */
		void SetFrom(float k1, float k2, float p1, float p2, float k3, float k4=0, float k5=0, float k6=0)
		{
            params[0] = k1;
            params[1] = k2;
            params[2] = p1;
            params[3] = p2;
            params[4] = k3;
            params[5] = k4;
            params[6] = k5;
            params[7] = k6;
		}

		/** Setup from standard arguments. */
		void SetStandard()
		{
            // no distortion is assumed
			SetFrom(0,0,0,0,0);
		}
	};
}
