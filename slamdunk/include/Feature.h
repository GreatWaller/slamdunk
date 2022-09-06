#pragma once
#include "Common.h"

namespace slamdunk {

    struct Feature {
    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        using Ptr = std::shared_ptr<Feature>;

        //std::weak_ptr<Frame> frame_;         // ���и�feature��frame
        cv::KeyPoint position_;              // 2D��ȡλ��
        //std::weak_ptr<MapPoint> map_point_;  // ������ͼ��

        bool is_outlier_ = false;       // �Ƿ�Ϊ�쳣��
        bool is_on_left_image_ = true;  // ��ʶ�Ƿ�������ͼ��falseΪ��ͼ

    public:
        Feature() {}

        Feature(const cv::KeyPoint& kp):position_(kp){}

        /*Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& kp)
            : frame_(frame), position_(kp) {}*/

    };
}