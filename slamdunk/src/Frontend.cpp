#include "Frontend.h"
#include "Config.h"
#include "Feature.h"
#include <opencv2/opencv.hpp>

namespace slamdunk {

	Frontend::Frontend() {
		m_detector = cv::GFTTDetector::create(
			Config::Get<int>("num_features"), 0.01, 20);
		m_numFeatures = Config::Get<int>("num_features");
		m_numFeaturesInit = Config::Get<int>("num_features_init");

	}

	bool Frontend::AddFrame(Frame::Ptr frame) {
		m_currentFrame = frame;

		switch (m_status) {
		case FrontendStatus::INITING:
			Init();
			break;
		case FrontendStatus::TRACKING_GOOD:
		case FrontendStatus::TRACKING_BAD:
			Track();
			break;
		case FrontendStatus::LOST:
			//Reset();
			break;
		}

		m_lastFrame = m_currentFrame;
		return true;
	}

	int Frontend::DetectFeatures() {
		cv::Mat mask(m_currentFrame->left_img_.size(), CV_8UC1, 255);
		// features by tracking last frame. No need to detect again. so mask it.
		for (auto& feature : m_currentFrame->features_left_) {
			cv::rectangle(mask, feature->keyPoint.pt - cv::Point2f(10, 10),
				feature->keyPoint.pt + cv::Point2f(10, 10),
				0, -1
			);
		}

		std::vector<cv::KeyPoint> keypoints;
		m_detector->detect(m_currentFrame->left_img_, keypoints, mask);

		int countDetected = 0;
		for (auto& keypoint : keypoints)
		{
			m_currentFrame->features_left_.push_back(Feature::Ptr(new
				Feature(keypoint)));
			countDetected++;

		}
		LOG_CORE_INFO("Detect {} new features.", countDetected);
		return countDetected;
	}

	/// <summary>
	/// 通过光流法找到与上一帧中feature的坐标点
	/// </summary>
	/// <returns></returns>
	int Frontend::TrackLastFrame() {

		std::vector<cv::Point2f> kps_last, kps_current;
		for (auto& kp : m_lastFrame->features_left_)
		{
			kps_last.push_back(kp->keyPoint.pt);
		}

		// 光流法
		std::vector<uchar> status;
		cv::Mat error;

		cv::calcOpticalFlowPyrLK(
			m_lastFrame->left_img_, m_currentFrame->left_img_,
			kps_last, kps_current,
			status, error, cv::Size(11, 11), 3,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01)
			,cv::OPTFLOW_LK_GET_MIN_EIGENVALS
		);

		int numGoodPoints = 0;
		for (size_t i = 0; i < status.size(); i++)
		{
			if (status[i])
			{
				cv::KeyPoint kp(kps_current[i], 7.0f);
				Feature::Ptr feature(new Feature(kp));

				m_currentFrame->features_left_.push_back(feature);
				numGoodPoints++;
			}
		}
		LOG_CORE_INFO("Find {} Good points in the current frame.", numGoodPoints);
		return numGoodPoints;
		
	}

	bool Frontend::Init() {
		int numFeaturesLeft = DetectFeatures();

		if (numFeaturesLeft< m_numFeaturesInit)
		{
			return false;
		}
		m_status = FrontendStatus::TRACKING_GOOD;
		return true;
	}

	bool Frontend::Track() {
		int numTrackLast = TrackLastFrame();

		// TODO: calculate inliners
		int trackingInliers = numTrackLast;

		if (trackingInliers > m_numFeaturesTrackingOK)
		{
			m_status = FrontendStatus::TRACKING_GOOD;
		}
		else if (trackingInliers > m_numFeaturesTrackingBad)
		{
			m_status = FrontendStatus::TRACKING_BAD;
		}
		else
		{
			m_status = FrontendStatus::LOST;
		}

		InsertKeyFrame();

		return true;
	}

	bool Frontend::InsertKeyFrame() {
		if (m_trackingInliers> m_numFeaturesNeededForKeyFrame)
		{
			return false;
		}
		m_currentFrame->SetKeyFrame();
		// TODO: map

		LOG_CORE_INFO("Set frame {} as key frame {}", m_currentFrame->id_,
			m_currentFrame->keyframe_id_);

		DetectFeatures();

		return true;
	}

	int Frontend::EstimateCurrentPose() {

	}
}