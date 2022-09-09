#pragma once

#include "Base.h"
#include "Frame.h"
#include "Common.h"

#include <opencv2/features2d.hpp>

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

	private:
		
		bool Track();
		bool Init();
		int TrackLastFrame();
		bool InsertKeyFrame();
		int DetectFeatures();
		/// <summary>
		/// estimate camera pose and return inliers
		/// </summary>
		/// <returns></returns>
		int EstimateCurrentPose();

	private:
		Ref<Frame> m_lastFrame;
		Ref<Frame> m_currentFrame;

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
	};
}