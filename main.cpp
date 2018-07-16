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

    /**
    cv::cv::Mat src;

    cv::VideoCapture capture;
    if (!capture.open(-1))
        return 0;

    QrDetector detector;
    ImageTransformer transformer;
    std::vector<cv::Point> corners;

    bool detected = false;

    while (capture.read(src)) {
        if (src.empty())
            break;

        cv::imshow("Source", src);

        /**
        if (!detected) {
            if (detector.findQr(src, corners)) {
                transformer = ImageTransformer(corners);
                detected = true;
            }
        } else {

            cv::cv::Mat transformed;
            transformer.transform(src, transformed);

            cv::imshow("Transform", transformed);
        }


    }*/


    cv::Mat source = cv::imread(TEST_IMAGE);
    cv::Mat canvas = source;
    show(source, "Source");

    QrDetector detector;
    std::vector<cv::Point> corners;
    detector.findQr(source, corners);

    cv::line(canvas, corners[0], corners[1], cv::Scalar(0,255,0));
    cv::line(canvas, corners[0], corners[2], cv::Scalar(0,255,0));
    cv::line(canvas, corners[3], corners[1], cv::Scalar(0,255,0));
    cv::line(canvas, corners[3], corners[2], cv::Scalar(0,255,0));
    show(canvas, "Corners");

    ImageTransformer transformer(corners);

    cv::Mat destination;
    transformer.transform(source, destination);

    show(destination, "Ok");

    /**
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
    cv::cv::Mat transformed;
    transformer.transform(src, transformed);
    show(transformed, "Perspective transformed");
    */
/**
    int alpha_ = 90, beta_ = 90, gamma_ = 90;
    int f_ = 500, dist_ = 500;

    double focalLength, dist, alpha, beta, gamma;

    alpha =((double)alpha_ -90) * CV_PI/180;
    beta =((double)beta_ -90) * CV_PI/180;
    gamma =((double)gamma_ -90) * CV_PI/180;
    focalLength = (double)f_;
    dist = (double)dist_;

    cv::Size image_size = source.size();
    double w = (double)image_size.width, h = (double)image_size.height;


    // Projecion matrix 2D -> 3D
    cv::Mat A1(4, 3, CV_64F);
    A1.at<float>(0, 0) = 1;
    A1.at<float>(0, 1) = 0;
    A1.at<float>(0, 2) = -w/2;
    A1.at<float>(1, 0) = 0;
    A1.at<float>(1, 1) = 1;
    A1.at<float>(1, 2) = -h/2;
    A1.at<float>(2, 0) = 0;
    A1.at<float>(2, 1) = 0;
    A1.at<float>(2, 2) = 0;
    A1.at<float>(3, 0) = 0;
    A1.at<float>(3, 1) = 0;
    A1.at<float>(3, 2) = 1;


    // Rotation matrices Rx, Ry, Rz
    cv::Mat RX(4, 4, CV_64F);
    RX.at<float>(0, 0) = 1;
    RX.at<float>(0, 1) = 0;
    RX.at<float>(0, 2) = 0;
    RX.at<float>(0, 3) = 0;
    RX.at<float>(1, 0) = 0;
    RX.at<float>(1, 1) = cos(alpha);
    RX.at<float>(1, 2) = -sin(alpha);
    RX.at<float>(1, 3) = 0;
    RX.at<float>(2, 0) = 0;
    RX.at<float>(2, 1) = sin(alpha);
    RX.at<float>(2, 2) = cos(alpha);
    RX.at<float>(2, 3) = 0;
    RX.at<float>(3, 0) = 0;
    RX.at<float>(3, 1) = 0;
    RX.at<float>(3, 2) = 0;
    RX.at<float>(3, 3) = 1;


    cv::Mat RY(4, 4, CV_64F);
    RY.at<float>(0, 0) = cos(beta);
    RY.at<float>(0, 1) = 0;
    RY.at<float>(0, 2) = -sin(beta);
    RY.at<float>(0, 3) = 0;
    RY.at<float>(1, 0) = 0;
    RY.at<float>(1, 1) = 1;
    RY.at<float>(1, 2) = 0;
    RY.at<float>(1, 3) = 0;
    RY.at<float>(2, 0) = sin(beta);
    RY.at<float>(2, 1) = 0;
    RY.at<float>(2, 2) = cos(beta);
    RY.at<float>(2, 3) = 0;
    RY.at<float>(3, 0) = 0;
    RY.at<float>(3, 1) = 0;
    RY.at<float>(3, 2) = 0;
    RY.at<float>(3, 3) = 1;


    cv::Mat RZ(4, 4, CV_64F);
    RZ.at<float>(0, 0) = cos(gamma);
    RZ.at<float>(0, 1) = -sin(gamma);
    RZ.at<float>(0, 2) = 0;
    RZ.at<float>(0, 3) = 0;
    RZ.at<float>(1, 0) = sin(gamma);
    RZ.at<float>(1, 1) = cos(gamma);
    RZ.at<float>(1, 2) = 0;
    RZ.at<float>(1, 3) = 0;
    RZ.at<float>(2, 0) = 0;
    RZ.at<float>(2, 1) = 0;
    RZ.at<float>(2, 2) = 1;
    RZ.at<float>(2, 3) = 0;
    RZ.at<float>(3, 0) = 0;
    RZ.at<float>(3, 1) = 0;
    RZ.at<float>(3, 2) = 0;
    RZ.at<float>(3, 3) = 1;

    // R - rotation matrix
    cv::Mat R = RX * RY * RZ;

    // T - translation matrix
    cv::Mat T(4, 4, CV_64F);
    T.at<float>(0, 0) = 1;
    T.at<float>(0, 1) = 0;
    T.at<float>(0, 2) = 0;
    T.at<float>(0, 3) = 0;
    T.at<float>(1, 0) = 0;
    T.at<float>(1, 1) = 1;
    T.at<float>(1, 2) = 0;
    T.at<float>(1, 3) = 0;
    T.at<float>(2, 0) = 0;
    T.at<float>(2, 1) = 0;
    T.at<float>(2, 2) = 1;
    T.at<float>(2, 3) = dist;
    T.at<float>(3, 0) = 0;
    T.at<float>(3, 1) = 0;
    T.at<float>(3, 2) = 0;
    T.at<float>(3, 3) = 1;

    // K - intrinsic matrix
    cv::Mat K(3, 4, CV_64F);
    K.at<float>(0, 0) = focalLength;
    K.at<float>(0, 1) = 0;
    K.at<float>(0, 2) = w/2;
    K.at<float>(0, 3) = 0;
    K.at<float>(1, 0) = 0;
    K.at<float>(1, 1) = focalLength;
    K.at<float>(1, 2) = h/2;
    K.at<float>(1, 3) = 0;
    K.at<float>(2, 0) = 0;
    K.at<float>(2, 1) = 0;
    K.at<float>(2, 2) = 1;
    K.at<float>(2, 3) = 0;

    cv::Mat transformationMat = K * (T * (R * A1));

    warpPerspective(source, destination, transformationMat, image_size, CV_INTER_CUBIC | CV_WARP_INVERSE_MAP);

    show(destination, "Transform");
*/
    return 0;
}