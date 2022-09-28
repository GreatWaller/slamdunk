#pragma once

#include "Log.h"
#include <iostream>
#include <memory>
#include <mutex>
#include <map>
#include <unordered_map>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using Mat33 = Eigen::Matrix<double, 3, 3>;
using Mat34= Eigen::Matrix<double, 3, 4> ;
using  Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;

// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using SE3 = Sophus::SE3d ;
using SO3 = Sophus::SO3d ;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;

