// SlamDunk.cpp: 定义应用程序的入口点。
//

#include "SlamDunk.h"
#include "Config.h"

using namespace std;

int test()
{
	cout << "Hello CMake." << endl;
	return 0;
}


namespace slamdunk {
	VisualOdometry::VisualOdometry(const std::string& configFile):
		configPath(configFile){}

	bool VisualOdometry::Init() {
		if (Config::SetConfigFile(configPath)==false)
		{
			return false;
		}
		dataset = Ref<Dataset>(new Dataset(Config::Get<std::string>("dataset_dir")));
		if (!dataset->Init()) {
			return false;
		}

		frontend = Ref<Frontend>(new Frontend());

		return true;
	}

	bool VisualOdometry::Step() {
		auto frame = dataset->NextFrame();
		if (frame == nullptr)
		{
			return false;
		}

		return frontend->AddFrame(frame);
	}

	bool VisualOdometry::Run() {
		while (true)
		{
			if (Step()==false)
			{
				break;
			}
		}

		LOG_CORE_INFO("VO exit.");
	}
}
