#pragma once

#include "Log.h"
#include <iostream>
#include <memory>
#include <mutex>


// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using Mat33 = Eigen::Matrix<double, 3, 3>;
using  Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;

// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using SE3 = Sophus::SE3d ;
using SO3 = Sophus::SO3d ;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;

