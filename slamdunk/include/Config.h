#pragma once

#include "Common.h"
namespace slamdunk {
	class Config {
	public:
		Config() = default;
		~Config();

		// access parameter
		template<typename T>
		static T Get(const std::string& key) {
			return static_cast<T>(Config::s_config->m_fs[key]);
		}

		// set a config file
		static bool SetConfigFile(const std::string& filename);

	private:
		static Ref<Config> s_config;
		cv::FileStorage m_fs;
	};
}