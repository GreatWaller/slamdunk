#include "Frontend.h"
#include "Config.h"
#include "Feature.h"
#include <opencv2/opencv.hpp>
#include "Optimizer.h"
#include "Algorithm.h"

namespace slamdunk {

	Frontend::Frontend() {
		m_detector = cv::GFTTDetector::create(
			Config::Get<int>("num_features"), 0.01, 20);
		m_numFeatures = Config::Get<int>("num_features");
		m_numFeaturesInit = Config::Get<int>("num_features_init");

	}

	void Frontend::SetCameras(Camera::Ptr leftCamera, Camera::Ptr rightCamera) {
		pLeftCamera = leftCamera;
		pRightCamera = rightCamera;
	}

	bool Frontend::AddFrame(Frame::Ptr frame) {
		mCurrentFrame = frame;

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

		mLastFrame = mCurrentFrame;
		return true;
	}

	int Frontend::DetectFeatures() {
		cv::Mat mask(mCurrentFrame->left_img_.size(), CV_8UC1, 255);
		// features by tracking last frame. No need to detect again. so mask it.
		for (auto& feature : mCurrentFrame->features_left_) {
			cv::rectangle(mask, feature->keyPoint.pt - cv::Point2f(10, 10),
				feature->keyPoint.pt + cv::Point2f(10, 10),
				0, -1
			);
		}

		std::vector<cv::KeyPoint> keypoints;
		m_detector->detect(mCurrentFrame->left_img_, keypoints, mask);

		int countDetected = 0;
		for (auto& keypoint : keypoints)
		{
			mCurrentFrame->features_left_.push_back(Feature::Ptr(new
				Feature(mCurrentFrame, keypoint)));
			countDetected++;

		}
		LOG_CORE_INFO("Detect {} new features.", countDetected);
		return countDetected;
	}

	int Frontend::FindFeaturesInRight() {
		// use LK Flow to estimate points in the right image.
		std::vector<cv::Point2f> kps_left, kps_right;
		for (auto& kp : mCurrentFrame->features_left_)
		{
			kps_left.push_back(kp->keyPoint.pt);
			auto mp = kp->map_point_.lock();
			if (mp)
			{
				auto px = pRightCamera->world2camera(mp->pos_, mCurrentFrame->Pose());
				kps_right.push_back(cv::Point2f(px[0], px[1]));
			}
			else
			{
				// use same pixel in left image
				kps_right.push_back(kp->keyPoint.pt);
			}
		}

		std::vector<uchar> status;
		Mat error;
		cv::calcOpticalFlowPyrLK(mCurrentFrame->left_img_,
			mCurrentFrame->right_img_, kps_left, kps_right, status, error,
			cv::Size(11, 11), 3,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
			cv::OPTFLOW_USE_INITIAL_FLOW);

		int numGoodPoints = 0;
		for (size_t i = 0; i < status.size(); i++)
		{
			if (status[i])
			{
				cv::KeyPoint kp(kps_right[i], 7);
				Feature::Ptr f(new Feature(mCurrentFrame, kp));
				f->is_on_left_image_ = false;
				mCurrentFrame->features_right_.push_back(f);
				numGoodPoints++;
			}
			else
			{
				mCurrentFrame->features_right_.push_back(nullptr);
			}
		}

		LOG_INFO("Find {} in the right image.", numGoodPoints);
		return numGoodPoints;
	}

	int Frontend::TriangulateNewPoints() {
		std::vector<SE3> poses{
			pLeftCamera->pose(), pRightCamera->pose()
		};
		SE3 current_pose_Twc = mCurrentFrame->Pose().inverse();
		int cnt_triangulated_pts = 0;
		for (size_t i = 0; i < mCurrentFrame->features_left_.size(); i++)
		{
			if (mCurrentFrame->features_left_[i]->map_point_.expired() &&
				mCurrentFrame->features_right_[i] != nullptr)
			{
				std::vector<Vec3> points{
					pLeftCamera->pixel2camera(
						Vec2(mCurrentFrame->features_left_[i]->keyPoint.pt.x,
							mCurrentFrame->features_left_[i]->keyPoint.pt.y)),
					pRightCamera->pixel2camera(
						Vec2(mCurrentFrame->features_right_[i]->keyPoint.pt.x,
							mCurrentFrame->features_right_[i]->keyPoint.pt.y)
					)
				};

				Vec3 pWorld = Vec3::Zero();
				if (triangulation(poses, points, pWorld))
				{
					auto new_map_point = MapPoint::CreateNewMappoint();
					pWorld = current_pose_Twc * pWorld;
					new_map_point->SetPos(pWorld);

					// for backend optimization
					new_map_point->AddObservation(mCurrentFrame->features_left_[i]);
					new_map_point->AddObservation(mCurrentFrame->features_right_[i]);

					mCurrentFrame->features_left_[i]->map_point_ = new_map_point;
					mCurrentFrame->features_right_[i]->map_point_ = new_map_point;

					// map
					pMap->InsertMapPoint(new_map_point);
					cnt_triangulated_pts++;
				}
			}
		}
		LOG_INFO("New Landmarks: {}", cnt_triangulated_pts);
		return cnt_triangulated_pts;
	}

	/// <summary>
	/// 通过光流法找到与上一帧中feature的坐标点
	/// </summary>
	/// <returns></returns>
	int Frontend::TrackLastFrame() {

		std::vector<cv::Point2f> kps_last, kps_current;
		for (auto& kp : mLastFrame->features_left_)
		{
			if (kp->map_point_.lock())
			{
				auto mp = kp->map_point_.lock();
				auto px = pLeftCamera->world2pixel(mp->pos_, mCurrentFrame->Pose());
				kps_last.push_back(kp->keyPoint.pt);
				kps_current.push_back(cv::Point2f(px[0], px[1]));
			}
			else
			{
				kps_last.push_back(kp->keyPoint.pt);
				kps_current.push_back(kp->keyPoint.pt);

			}

		}

		// 光流法
		std::vector<uchar> status;
		cv::Mat error;

		cv::calcOpticalFlowPyrLK(
			mLastFrame->left_img_, mCurrentFrame->left_img_,
			kps_last, kps_current,
			status, error, cv::Size(11, 11), 3,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01)
			, cv::OPTFLOW_LK_GET_MIN_EIGENVALS
		);

		int numGoodPoints = 0;
		for (size_t i = 0; i < status.size(); i++)
		{
			if (status[i])
			{
				cv::KeyPoint kp(kps_current[i], 7.0f);
				Feature::Ptr feature(new Feature(mCurrentFrame, kp));
				// for pose estimate
				feature->map_point_ = mLastFrame->features_left_[i]->map_point_;

				mCurrentFrame->features_left_.push_back(feature);

				numGoodPoints++;
			}
		}
		LOG_CORE_INFO("Find {} Good points in the current frame.", numGoodPoints);
		return numGoodPoints;

	}

	bool Frontend::Init() {
		int numFeaturesLeft = DetectFeatures();

		int numFeaturesRight = FindFeaturesInRight();

		if (numFeaturesRight < m_numFeaturesInit)
		{
			return false;
		}
		bool buildMapSucceed = BuildInitMap();
		if (buildMapSucceed)
		{
			m_status = FrontendStatus::TRACKING_GOOD;

			return true;
		}
		return false;
	}

	bool Frontend::Track() {
		if (mLastFrame)
		{
			mCurrentFrame->SetPose(m_relativeMotion * mLastFrame->Pose());
		}

		int numTrackLast = TrackLastFrame();
		EstimateCurrentPose();

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

		m_relativeMotion = mCurrentFrame->Pose() * mLastFrame->Pose().inverse();

		return true;
	}

	bool Frontend::InsertKeyFrame() {
		if (m_trackingInliers > m_numFeaturesNeededForKeyFrame)
		{
			return false;
		}
		mCurrentFrame->SetKeyFrame();
		pMap->InsertKeyFrame(mCurrentFrame);

		LOG_CORE_INFO("Set frame {} as key frame {}", mCurrentFrame->id_,
			mCurrentFrame->keyframe_id_);
		// TODO: map
		SetObservationForKeyFrame();

		DetectFeatures();

		FindFeaturesInRight();
		TriangulateNewPoints();

		pBackend->UpdateMap();

		return true;
	}

	void Frontend::SetObservationForKeyFrame() {
		for (auto& feat : mCurrentFrame->features_left_)
		{
			auto mp = feat->map_point_.lock();
			if (mp)
			{
				mp->AddObservation(feat);
			}
		}
	}

	int Frontend::EstimateCurrentPose() {
		using BlockSolverType = g2o::BlockSolver_6_3;

		using LinearSolverType =
			g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

		auto linearSolver = new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>();
		auto blockSolver = new BlockSolverType(
			std::unique_ptr<LinearSolverType>(linearSolver));

		auto solver = new g2o::OptimizationAlgorithmLevenberg(
			std::unique_ptr<BlockSolverType>(blockSolver));

		//auto solver = new g2o::OptimizationAlgorithmLevenberg(
		//	g2o::make_unique<BlockSolverType>(
		//		g2o::make_unique<LinearSolverType>()));

		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm(solver);

		// vertex
		VertexPose* vertexPose = new VertexPose();
		vertexPose->setId(0);
		vertexPose->setEstimate(mCurrentFrame->Pose());
		optimizer.addVertex(vertexPose);


		//edge
		Mat33 K = pLeftCamera->K();
		int index = 1;
		std::vector<EdgeProjectionPoseOnly*> edges;
		std::vector<Feature::Ptr> features;
		for (size_t i = 0; i < mCurrentFrame->features_left_.size(); i++)
		{
			auto mp = mCurrentFrame->features_left_[i]->map_point_.lock();
			if (mp)
			{
				features.push_back(mCurrentFrame->features_left_[i]);
				EdgeProjectionPoseOnly* edge = new EdgeProjectionPoseOnly(mp->pos_, K);
				edge->setId(index);
				edge->setVertex(0, vertexPose);
				edge->setMeasurement(
					Vec2(mCurrentFrame->features_left_[i]->keyPoint.pt.x,
						mCurrentFrame->features_left_[i]->keyPoint.pt.y)
				);
				edge->setInformation(Eigen::Matrix2d::Identity());
				edge->setRobustKernel(new g2o::RobustKernelHuber);
				edges.push_back(edge);
				optimizer.addEdge(edge);
				index++;
			}
		}
		// estimate pose
		// set the initial value to optimize
		vertexPose->setEstimate(mCurrentFrame->Pose());

		optimizer.initializeOptimization();
		optimizer.optimize(10);

		mCurrentFrame->SetPose(vertexPose->estimate());
		LOG_INFO("Current Pose: {}", mCurrentFrame->Pose().matrix());

		return 0;
	}

	bool Frontend::BuildInitMap() {
		std::vector<SE3> poses{
			pLeftCamera->pose(), pRightCamera->pose()
		};
		SE3 current_pose_Twc = mCurrentFrame->Pose().inverse();
		int cnt_triangulated_pts = 0;
		for (size_t i = 0; i < mCurrentFrame->features_left_.size(); i++)
		{
			if (mCurrentFrame->features_right_[i]==nullptr)
			{
				continue;
			}
			std::vector<Vec3> points{
				pLeftCamera->pixel2camera(
					Vec2(mCurrentFrame->features_left_[i]->keyPoint.pt.x,
						mCurrentFrame->features_left_[i]->keyPoint.pt.y)),
				pRightCamera->pixel2camera(
					Vec2(mCurrentFrame->features_right_[i]->keyPoint.pt.x,
						mCurrentFrame->features_right_[i]->keyPoint.pt.y)
				)
			};

			Vec3 pWorld = Vec3::Zero();
			if (triangulation(poses, points, pWorld) && pWorld[2]>0)
			{
				auto new_map_point = MapPoint::CreateNewMappoint();
				pWorld = current_pose_Twc * pWorld;
				new_map_point->SetPos(pWorld);

				new_map_point->AddObservation(mCurrentFrame->features_left_[i]);
				new_map_point->AddObservation(mCurrentFrame->features_right_[i]);

				mCurrentFrame->features_left_[i]->map_point_ = new_map_point;
				mCurrentFrame->features_right_[i]->map_point_ = new_map_point;

				pMap->InsertMapPoint(new_map_point);
				cnt_triangulated_pts++;
			}

		}
		mCurrentFrame->SetKeyFrame();
		pMap->InsertKeyFrame(mCurrentFrame);
		// TODO: backend update map
		pBackend->UpdateMap();
		LOG_INFO("Initial map created with {} map points.", cnt_triangulated_pts);
		return true;
	}
}