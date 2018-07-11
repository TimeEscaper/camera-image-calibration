//
// Created by sibirsky on 11.07.18.
//

#ifndef CAMERA_IMAGE_CALIBRATION_IMAGETRANSFORMER_H
#define CAMERA_IMAGE_CALIBRATION_IMAGETRANSFORMER_H


#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

class ImageTransformer {

public:
    ImageTransformer(const std::vector<cv::Point> &rectPoints);
    ImageTransformer(ImageTransformer &other);

    void transform(const cv::Mat &image, cv::Mat &transformed);

private:
    void sortPoints(const std::vector<cv::Point> &points, std::vector<cv::Point> &sorted);

    std::vector<cv::Point> points;
    cv::Mat transformMat;
    int size;
};


#endif //CAMERA_IMAGE_CALIBRATION_IMAGETRANSFORMER_H
