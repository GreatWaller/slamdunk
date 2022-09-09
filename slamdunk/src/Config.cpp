#include "Config.h"

namespace slamdunk {

	Ref<Config> Config::s_config = nullptr;


	Config::~Config() {
		if (m_fs.isOpened())
		{
			m_fs.release();
		}
	}

	bool Config::SetConfigFile(const std::string& filename) {
		if (s_config == nullptr) {
			s_config = Ref<Config>(new Config);
		}
		s_config->m_fs = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
		if (s_config->m_fs.isOpened() ==false)
		{
			LOG_CORE_ERROR("Parameter file {0} does not exist.", filename);
			s_config->m_fs.release();
			return false;

		}
		return true;
	}
}