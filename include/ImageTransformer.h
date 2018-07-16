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
    ImageTransformer();
    ImageTransformer(const std::vector<cv::Point> &rectPoints);
    ImageTransformer(ImageTransformer &other);

    ImageTransformer& operator=(const ImageTransformer &other);

    static void sortPoints(const std::vector<cv::Point> &points, std::vector<cv::Point> &sorted);


    void transform(const cv::Mat &image, cv::Mat &transformed);

private:

    std::vector<cv::Point> points;
    cv::Mat transformMat;
    int size;
    double width, height;
};


#endif //CAMERA_IMAGE_CALIBRATION_IMAGETRANSFORMER_H
