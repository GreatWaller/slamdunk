#pragma once

#include "Base.h"

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace slamdunk {

	class Log
	{
	public:
		static void Init();

		static Ref<spdlog::logger>& GetCoreLogger() { return s_CoreLogger; }
		static Ref<spdlog::logger>& GetClientLogger() { return s_ClientLogger; }
	private:
		static Ref<spdlog::logger> s_CoreLogger;
		static Ref<spdlog::logger> s_ClientLogger;
	};

}

// Core log macros
#define LOG_CORE_TRACE(...)    ::slamdunk::Log::GetCoreLogger()->trace(__VA_ARGS__)
#define LOG_CORE_INFO(...)     ::slamdunk::Log::GetCoreLogger()->info(__VA_ARGS__)
#define LOG_CORE_WARN(...)     ::slamdunk::Log::GetCoreLogger()->warn(__VA_ARGS__)
#define LOG_CORE_ERROR(...)    ::slamdunk::Log::GetCoreLogger()->error(__VA_ARGS__)
#define LOG_CORE_CRITICAL(...) ::slamdunk::Log::GetCoreLogger()->critical(__VA_ARGS__)
					
// ClienLOG_og macros			
#define LOG_TRACE(...)         ::slamdunk::Log::GetClientLogger()->trace(__VA_ARGS__)
#define LOG_INFO(...)          ::slamdunk::Log::GetClientLogger()->info(__VA_ARGS__)
#define LOG_WARN(...)          ::slamdunk::Log::GetClientLogger()->warn(__VA_ARGS__)
#define LOG_ERROR(...)         ::slamdunk::Log::GetClientLogger()->error(__VA_ARGS__)
#define LOG_CRITICAL(...)      ::slamdunk::Log::GetClientLogger()->critical(__VA_ARGS__)