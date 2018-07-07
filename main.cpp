#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#define TEST_IMAGE "/home/sibirsky/calibration_images/a_mIC88p93c.jpg"

void show(cv::Mat image, const char* name) {
    cv::namedWindow(name);
    cv::imshow(name, image);
    cv::waitKey();
    cv::destroyWindow(name);
}

bool findQrMarks(const cv::Mat &image, std::vector<std::vector<cv::Point>> &markContours) {
    cv::Mat blured;
    cv::blur(image, blured, cv::Size(3, 3));

    cv::Mat gray;
    cv::cvtColor(blured, gray, CV_BGR2GRAY);

    cv::Mat bw;
    cv::adaptiveThreshold(gray, bw, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 51, 0);

    cv::Mat edges;
    cv::Canny(bw, edges, 50, 150, 5);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    int markCount, mark1, mark2, mark3;
    markCount = mark1 = mark2 = mark3 = 0;

    for (int i = 0; i < contours.size(); i++) {
        int k = i;
        int c = 0;

        while(hierarchy[k][2] != -1) {
            k = hierarchy[k][2] ;
            c++;
        }
        if(hierarchy[k][2] != -1)
            c++;

        if (c >= 5) {
            switch (markCount) {
                case 0:
                    mark1 = i;
                    break;
                case 1:
                    mark2 = i;
                    break;
                case 2:
                    mark3 = i;
                    break;
                default:
                    break;
            }
            markCount++;
        }
    }

    if (markCount >= 3) {
        markContours.clear();
        markContours.push_back(contours[mark1]);
        markContours.push_back(contours[mark2]);
        markContours.push_back(contours[mark3]);

        return true;
    }

    return false;
}

int main() {
    cv::Mat src = cv::imread(TEST_IMAGE);
    show(src, "Source");

    std::vector<std::vector<cv::Point>> markContours;
    if (findQrMarks(src, markContours))
        cv::drawContours(src, markContours, -1, cv::Scalar(255, 0, 0));

    show(src, "QR marks");

    return 0;
}