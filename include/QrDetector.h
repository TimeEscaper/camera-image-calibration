//
// Created by sibirsky on 11.07.18.
//

#ifndef CAMERA_IMAGE_CALIBRATION_QRDETECTOR_H
#define CAMERA_IMAGE_CALIBRATION_QRDETECTOR_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

class QrDetector {

public:
    bool findQr(const cv::Mat &image, std::vector<cv::Point> &corners);

private:
    bool findQrMarks(const cv::Mat &image, std::vector<std::vector<cv::Point>> &markContours);
    void findCorners(const std::vector<std::vector<cv::Point>> &contours,
                     std::vector<cv::Point> &corners);

};


#endif //CAMERA_IMAGE_CALIBRATION_QRDETECTOR_H
