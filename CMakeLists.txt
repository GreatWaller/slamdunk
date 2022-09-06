# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.8)
project ("SlamDunk")


# set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")
# set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++14 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories(${PROJECT_SOURCE_DIR}/slamdunk/include)

# 3rd packages
# Eigen
include_directories("/usr/include/eigen3")

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
# message("Sophus DIR: ${Sophus_INCLUDE_DIRS}")
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# spdlog
# Stand-alone build
# find_package(spdlog REQUIRED )
include_directories(${PROJECT_SOURCE_DIR}/vendor/spdlog/include)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/vendor/spdlog/build)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        spdlog
        )



# 包含子项目。
add_subdirectory ("slamdunk")
add_subdirectory("app")
