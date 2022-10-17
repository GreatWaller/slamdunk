// SlamDunk.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once

#include "Common.h"
#include "DataSet.h"
#include "Frontend.h"
#include "Backend.h"

int test();

namespace slamdunk {
	class VisualOdometry {
	public:
		using Ptr = Ref<VisualOdometry>;

		VisualOdometry(const std::string& configFile);
		bool Init();

		bool Step();
		void Run();

	private:
		std::string configPath;

		Ref<Dataset> dataset = nullptr;
		Ref<Frontend> frontend = nullptr;
		Ref<Backend> backend = nullptr;
		Ref<Map> map = nullptr;
	};
}
