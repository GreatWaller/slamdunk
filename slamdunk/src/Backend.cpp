#include "Backend.h"
#include "Optimizer.h"
#include "Feature.h"
#include "Algorithm.h"

namespace slamdunk {
	Backend::Backend() {
		isBackendRunning.store(true);
		mBackendThread = std::thread(std::bind(&Backend::BackendLoop, this));
	}

	void Backend::SetCameras(Camera::Ptr left, Camera::Ptr right) {
		pLeftCamera = left;
		pRightCamera = right;
	}
	void Backend::UpdateMap() {
		std::lock_guard<std::mutex> lock{ lmMap };
		lcMap.notify_one();
	}

	void Backend::Stop() {
		isBackendRunning.store(false);
		lcMap.notify_one();
		mBackendThread.join();
	}

	void Backend::BackendLoop() {
		while (isBackendRunning.load())
		{
			std::unique_lock<std::mutex> lock(lmMap);
			lcMap.wait(lock);

			// optimized only by active landmarks and frames
			Map::KeyFramesType activeFrames = pMap->GetActiveKeyFrames();
			Map::LandmarksType activeLandmarks = pMap->GetActiveMapPoints();

			Optimize(activeFrames, activeLandmarks);
		}
	}

	void Backend::Optimize(Map::KeyFramesType& keyframes,
		Map::LandmarksType& landmarks) {
		//setup g2o
		using BlockSolverType = g2o::BlockSolver_6_3;
		using LinearSolverType = g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>;

		auto solver = new g2o::OptimizationAlgorithmLevenberg(
			g2o::make_unique<BlockSolverType>(
				g2o::make_unique<LinearSolverType>()
				)
		);
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm(solver);

		// vertex pose
		std::map<unsigned long, VertexPose*> vertices;
		unsigned long max_kf_id = 0;
		for (auto& keyFrame : keyframes)
		{
			auto kf = keyFrame.second;
			VertexPose* vertexPose = new VertexPose();
			vertexPose->setId(kf->keyframe_id_);
			vertexPose->setEstimate(kf->Pose());
			optimizer.addVertex(vertexPose);
			if (kf->keyframe_id_>max_kf_id)
			{
				max_kf_id = kf->keyframe_id_;
			}
			vertices.insert({ kf->keyframe_id_, vertexPose });
		}

		// vertex landmark
		std::map<unsigned long, VertexXYZ*> verticesLandmarks;

		Mat33 K = pLeftCamera->K();
		SE3 leftExtrinsic = pLeftCamera->pose();
		SE3 rightExtrinsic = pRightCamera->pose();

		// edges
		int index = 1;
		double chi2_th = 5.991; // robust kernel 
		std::map<EdgeProjection*, Feature::Ptr> edges_and_features;

		for (auto& landmark : landmarks)
		{
			if (landmark.second->is_outlier_)
			{
				continue;
			}
			unsigned long landmarkId = landmark.second->id_;
			auto observations = landmark.second->GetObs();
			for (auto& obs : observations)
			{
				auto feat = obs.lock();
				if (feat==nullptr)
				{
					continue;
				}
				if (feat->is_outlier_ || feat->frame_.lock()==nullptr)
				{
					continue;
				}
				auto frame = feat->frame_.lock();
				EdgeProjection* edge = nullptr;
				if (feat->is_on_left_image_)
				{
					edge = new EdgeProjection(K, leftExtrinsic);
				}
				else
				{
					edge = new EdgeProjection(K, rightExtrinsic);
				}

				// if landmark is not added, add it as a vertex
				if (verticesLandmarks.find(landmarkId)==verticesLandmarks.end())
				{
					VertexXYZ* v = new VertexXYZ;
					v->setEstimate(landmark.second->Pos());
					v->setId(landmarkId + max_kf_id + 1);
					v->setMarginalized(true);
					verticesLandmarks.insert({ landmarkId,v });
					optimizer.addVertex(v);
				}

				edge->setId(index);
				edge->setVertex(0, vertices.at(frame->keyframe_id_));
				edge->setVertex(1, verticesLandmarks.at(landmarkId));
				edge->setMeasurement(toVec2(feat->keyPoint.pt));
				edge->setInformation(Mat22::Identity());
				auto rk = new g2o::RobustKernelHuber();
				rk->setDelta(chi2_th);
				edge->setRobustKernel(rk);

				edges_and_features.insert({ edge,feat });

				optimizer.addEdge(edge);
				index++;
			}
		}

		optimizer.initializeOptimization();
		optimizer.optimize(10);

		// outlier
		int cnt_outlier = 0, cnt_inlier = 0;
		int iteration = 0;
		while (iteration<5)
		{
			cnt_outlier = 0;
			cnt_inlier = 0;
			
			// determine if we want to adjust the outlier threshold
			for (auto& ef : edges_and_features)
			{
				if (ef.first->chi2() >chi2_th)
				{
					cnt_outlier++;

				}
				else
				{
					cnt_inlier++;
				}
			}

			double inlierRatio = cnt_inlier / double(cnt_inlier + cnt_outlier);
			if (inlierRatio>0.5)
			{
				break;
			}
			else
			{
				chi2_th *= 2;
				iteration++;
			}
		}
		for (auto& ef : edges_and_features)
		{
			if (ef.first->chi2()>chi2_th)
			{
				ef.second->is_outlier_ = true;
				// remove the observation
				ef.second->map_point_.lock()->RemoveObservation(ef.second);
			}
			else
			{
				ef.second->is_outlier_ = false;
			}
		}

		LOG_INFO("Outlier/Inlier in optimizaation: {0}/{1}", cnt_outlier, cnt_inlier);

		// set back
		for (auto& v : vertices)
		{
			keyframes.at(v.first)->SetPose(v.second->estimate());
		}
		for (auto& v : verticesLandmarks)
		{
			landmarks.at(v.first)->SetPos(v.second->estimate());
		}
	}
}