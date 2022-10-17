#include "Map.h"

namespace slamdunk {
	
	void Map::InsertKeyFrame(Frame::Ptr frame) {
		pCurrentFrame = frame;
		// not found
		if (mKeyFrames.find(frame->keyframe_id_)==mKeyFrames.end())
		{
			mKeyFrames.insert(std::make_pair(frame->keyframe_id_, frame));
			mActiveKeyFrames.insert(std::make_pair(frame->keyframe_id_, frame));
		}
		else
		{
			// found and update
			mKeyFrames[frame->keyframe_id_] = frame;
			mActiveKeyFrames[frame->keyframe_id_] = frame;
		}
		if (mActiveKeyFrames.size()>cNumActiveKeyframes)
		{
			RemoveOldKeyframe();
		}
	}

	void Map::InsertMapPoint(MapPoint::Ptr mapPoint) {
		if (mLandmarks.find(mapPoint->id_)==mLandmarks.end())
		{
			mLandmarks.insert(std::make_pair(mapPoint->id_, mapPoint));
			mActiveLandmarks.insert(std::make_pair(mapPoint->id_, mapPoint));
		}
		else
		{
			mLandmarks[mapPoint->id_] = mapPoint;
			mActiveLandmarks[mapPoint->id_] = mapPoint;
		}
	}

	void Map::RemoveOldKeyframe() {
		// TODO: calculate the distance and remove too close to or far from current frame.

	}
}