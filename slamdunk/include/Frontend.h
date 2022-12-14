#pragma once

#include "Base.h"
#include "Frame.h"
#include "Common.h"
#include "Camera.h"
#include "Map.h"
#include <opencv2/features2d.hpp>
#include "Backend.h"

namespace slamdunk {
	enum class FrontendStatus
	{
		INITING, TRACKING_GOOD, TRACKING_BAD, LOST
	};

	class Frontend {
	public:
		Frontend();

		/// 外部接口，添加一个帧并计算其定位结果
		bool AddFrame(Frame::Ptr frame);

		inline void SetMap(Map::Ptr map) { pMap = map; }
		void SetCameras(Camera::Ptr leftCamera, Camera::Ptr rightCamera);
		void SetBackend(Backend::Ptr backend) { pBackend = backend; }
	private:
		
		bool Track();
		bool Init();
		int TrackLastFrame();
		bool InsertKeyFrame();
		void SetObservationForKeyFrame();
		int DetectFeatures();
		int FindFeaturesInRight();

		int TriangulateNewPoints();
		/// <summary>
		/// estimate camera pose and return inliers
		/// </summary>
		/// <returns></returns>
		int EstimateCurrentPose();

		bool BuildInitMap();

	private:
		Ref<Frame> mLastFrame;
		Ref<Frame> mCurrentFrame;

		SE3 m_relativeMotion;
		cv::Ptr<cv::GFTTDetector> m_detector;

		int m_numFeatures=200;
		int m_numFeaturesInit=100;
		int m_numFeaturesTrackingOK = 50;
		int m_numFeaturesTrackingBad = 20;

		// use for key frame
		int m_numFeaturesNeededForKeyFrame = 80;
		int m_trackingInliers = 0;

		FrontendStatus m_status = FrontendStatus::INITING;

		// camera
		Camera::Ptr pLeftCamera = nullptr;
		Camera::Ptr pRightCamera = nullptr;

		// map
		Map::Ptr pMap = nullptr;

		// backend
		Backend::Ptr pBackend = nullptr;

	};
}