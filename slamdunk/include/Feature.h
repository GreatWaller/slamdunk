#pragma once
#include "Common.h"
#include "MapPoint.h"
#include "Frame.h"

namespace slamdunk {

    class Feature {
    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        using Ptr = std::shared_ptr<Feature>;

        std::weak_ptr<Frame> frame_;         // 持有该feature的frame
        cv::KeyPoint keyPoint;              // 2D提取位置
        std::weak_ptr<MapPoint> map_point_;  // 关联地图点

        bool is_outlier_ = false;       // 是否为异常点
        bool is_on_left_image_ = true;  // 标识是否提在左图，false为右图

    public:
        Feature() {}

        //Feature(const cv::KeyPoint& kp):keyPoint(kp){}

        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& kp)
            : frame_(frame), keyPoint(kp) {}

    };
}