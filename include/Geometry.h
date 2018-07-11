//
// Created by sibirsky on 11.07.18.
//

#ifndef CAMERA_IMAGE_CALIBRATION_GEOMETRY_H
#define CAMERA_IMAGE_CALIBRATION_GEOMETRY_H


#include <opencv2/core/types.hpp>

namespace geom {

    double distance(cv::Point2f a, cv::Point2f b);
    double distance(cv::Point2f linePoint1, cv::Point2f linePoint2, cv::Point2f point);
    float cross(cv::Point2f v1,cv::Point2f v2);
    cv::Point center(std::vector<cv::Point> contour);
    bool intersectionPoint(cv::Point2f a1, cv::Point2f a2, cv::Point2f b1, cv::Point2f b2,
                           cv::Point2f& intersection);

}


#endif //CAMERA_IMAGE_CALIBRATION_GEOMETRY_H
