#pragma once

#include "Common.h"
#include "Feature.h"

namespace slamdunk {

	class MapPoint
	{
	public:
		using Ptr = Ref<MapPoint>;
		unsigned long id_ = 0;
		bool is_outlier_ = false;
		Vec3 pos_ = Vec3::Zero();
		std::mutex data_mutex_;
		int observed_times_ = 0;  // being observed by feature matching algo.
		std::list<std::weak_ptr<Feature>> observations_; // 说明：一个map点对应左右目两个特征点

		MapPoint() {}

		MapPoint(long id, Vec3 position);

		Vec3 Pos() {
			std::unique_lock<std::mutex> lck(data_mutex_);
			return pos_;
		}

		void SetPos(const Vec3& pos) {
			std::unique_lock<std::mutex> lck(data_mutex_);
			pos_ = pos;
		};

		static MapPoint::Ptr CreateNewMappoint();
		void RemoveObservation(std::shared_ptr<Feature> feature);

		std::list<std::weak_ptr<Feature>> GetObs() {
			std::lock_guard<std::mutex> lock(data_mutex_);
			return observations_;
		}

		void AddObservation(std::shared_ptr<Feature> feature) {
			std::lock_guard<std::mutex> lock(data_mutex_);
			observations_.push_back(feature);
			observed_times_++;
		}
	};
}