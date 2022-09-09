#include "SlamDunk.h"
#include "Feature.h"

#include "Log.h"
#include "DataSet.h"

#include "SlamDunk.h"

//using namespace slamdunk;
int main() {
	test();
	auto f = new slamdunk::Feature;


	slamdunk::Log::Init();

	LOG_INFO("SLAM DUNK {0} ({1}, {2})",1, 2, 3);

	// test dataset
	//slamdunk::Dataset dataset("/home/vector4d/xander/dataset/odometry/00");
	//auto frame= dataset.NextFrame();


	using namespace slamdunk;
	VisualOdometry vo("../config/config.yaml");
	if (!vo.Init()) {
		LOG_CORE_WARN("VisualOdometry Init Failed.");
		return 0;
	}
	vo.Run();
	return 0;
}