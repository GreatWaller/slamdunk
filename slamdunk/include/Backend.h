#pragma once
#include "Common.h"
#include "Map.h"
#include "Camera.h"
namespace slamdunk {
	class Backend {
	public:
		using Ptr = std::shared_ptr<Backend>;
	private:
		std::shared_ptr<Map> pMap;

		Camera::Ptr pLeftCamera = nullptr, pRightCamera = nullptr;

		std::mutex lmMap;

		std::thread mBackendThread;
		std::condition_variable lcMap;
		std::atomic<bool> isBackendRunning;

	public:
		Backend();

		inline void SetMap(std::shared_ptr<Map> map) { pMap = map; }
		void SetCameras(Camera::Ptr left, Camera::Ptr right);
		/// <summary>
		/// triggered by frontend 
		/// </summary>
		void UpdateMap();
		void Stop();

	private:
		void BackendLoop();

		void Optimize(Map::KeyFramesType& keyFrames, Map::LandmarksType& landmarks);
	};
}