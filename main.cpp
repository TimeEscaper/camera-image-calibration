#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "include/QrDetector.h"
#include "include/ImageTransformer.h"

#define TEST_IMAGE "/home/sibirsky/calibration_images/my_photo-5.jpg"


void show(cv::Mat image, const char* name) {
    cv::namedWindow(name);
    cv::imshow(name, image);
    cv::waitKey();
    cv::destroyWindow(name);
}

int main() {

    cv::Mat source = cv::imread(TEST_IMAGE);
    show(source, "Source");

    QrDetector detector;
    std::vector<cv::Point> corners;
    if (detector.findQr(source, corners)) {

        ImageTransformer transformer(corners);

        cv::Mat destination;
        transformer.transform(source, destination);

        show(destination, "Ok");
    }

    return 0;
}