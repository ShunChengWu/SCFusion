// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include <ORUtils/SE3Pose.h>

namespace FernRelocLib
{
	class PoseDatabase
	{
	public:
		struct PoseInScene
		{
			PoseInScene() {}
			PoseInScene(const ORUtils::SE3Pose & _pose, int _sceneIdx) : pose(_pose), sceneIdx(_sceneIdx) {}
			ORUtils::SE3Pose pose;
			int sceneIdx;
		};

		PoseDatabase();
		~PoseDatabase();

		void storePose(int id, const ORUtils::SE3Pose & pose, int sceneId);
		int numPoses() const { return (int)mPoses.size(); }

		const PoseInScene & retrievePose(int id) const { return mPoses[id]; }
		PoseInScene retrieveWAPose(int k, int ids[], float weights[]) const;

		void SaveToFile(const std::string &fileName);
		void LoadFromFile(const std::string &fileName);

	private:
		std::vector<PoseInScene> mPoses;
	};
}
