#pragma once
#include "Common.h"
#include "MapPoint.h"
#include "Frame.h"
#include <unordered_map>

namespace slamdunk {
	class Map {
	public:
		using Ptr = std::shared_ptr<Map>;
		using LandmarksType = std::unordered_map<unsigned long, MapPoint::Ptr>;
		using KeyFramesType = std::unordered_map<unsigned long, Frame::Ptr>;

		Map(){}

		void InsertKeyFrame(Frame::Ptr frame);
		void InsertMapPoint(MapPoint::Ptr mapPoint);

		LandmarksType GetAllMapPoints() {
			std::unique_lock<std::mutex> lock(mutex);
			return mLandmarks;
		}

		KeyFramesType GetAllKeyFrames() {
			std::unique_lock<std::mutex> lock(mutex);
			return mKeyFrames;
		}

		LandmarksType GetActiveMapPoints() {
			std::unique_lock<std::mutex> lock(mutex);
			return mActiveLandmarks;
		}

		KeyFramesType GetActiveKeyFrames() {
			std::unique_lock<std::mutex> lock(mutex);
			return mActiveKeyFrames;
		}
	private:
		void RemoveOldKeyframe();

	private:
		LandmarksType mLandmarks;
		LandmarksType mActiveLandmarks;
		KeyFramesType mKeyFrames;
		KeyFramesType mActiveKeyFrames;

		Frame::Ptr pCurrentFrame = nullptr;

		const int cNumActiveKeyframes = 7;
		std::mutex mutex;
	};
}