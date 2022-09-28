# CMakeList.txt: SlamDunk 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

# 将源代码添加到此项目的可执行文件。
add_library (slamdunk SHARED
	"src/SlamDunk.cpp" 
	"include/Common.h" 
	"include/MapPoint.h" 
	"include/Camera.h" 
	"src/Camera.cpp" 
	"src/Frame.cpp" 
	"src/DataSet.cpp" 
	"include/Log.h" 
	"include/Base.h" 
	"src/Log.cpp" 
	"src/MapPoint.cpp" 
	"include/Frontend.h" 
	"src/Frontend.cpp" 
	"include/Config.h" 
	"src/Config.cpp" "include/Optimizer.h" "include/Algorithm.h" "include/Map.h" "src/Map.cpp")

target_link_libraries(slamdunk
    ${THIRD_PARTY_LIBS})

