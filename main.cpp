#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "include/QrDetector.h"
#include "include/ImageTransformer.h"

#define TEST_IMAGE "/home/sibirsky/calibration_images/t55LVeq3esM.jpg"

void show(cv::Mat image, const char* name) {
    cv::namedWindow(name);
    cv::imshow(name, image);
    cv::waitKey();
    cv::destroyWindow(name);
}

int main() {
    cv::Mat src = cv::imread(TEST_IMAGE);
    cv::Mat canvas = src;
    show(src, "Source");

    QrDetector detector;
    std::vector<cv::Point> corners;
    if (!detector.findQr(src, corners))
        return 0;

    cv::line(canvas, corners[0], corners[1], cv::Scalar(0,255,0));
    cv::line(canvas, corners[0], corners[2], cv::Scalar(0,255,0));
    cv::line(canvas, corners[3], corners[1], cv::Scalar(0,255,0));
    cv::line(canvas, corners[3], corners[2], cv::Scalar(0,255,0));
    show(canvas, "Corners");

    ImageTransformer transformer(corners);
    cv::Mat transformed;
    transformer.transform(src, transformed);
    show(transformed, "Perspective transformed");

    return 0;
}