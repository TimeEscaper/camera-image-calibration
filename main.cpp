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

void closing(cv::Mat &image) {
    cv::Mat structElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    for (int i = 1; i <= 1; i++) {
        //cv::morphologyEx(image, image, cv::MORPH_CLOSE, structElement, cv::Point(-1,-1), i);
        cv::erode(image, image, structElement);
    }
}

int main() {
    cv::Mat src = cv::imread(TEST_IMAGE);
    //cv::resize(src, src, cv::Size(), 0.75, 0.75, CV_INTER_LINEAR);
    cv::blur(src, src, cv::Size(3, 3));
    show(src, "Source");

    cv::Mat graySource;
    cv::cvtColor(src, graySource, CV_BGR2GRAY);
    show(graySource, "Gray");

    cv::Mat bwSource;
    cv::adaptiveThreshold(graySource, bwSource, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 51, 0);
    //closing(bwSource);
    show(bwSource, "BW");

    cv::Mat edges;
    cv::Canny(bwSource, edges, 50, 150, 5);
    //closing(edges);
    show(edges, "Edges");

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    //cv::drawContours(src, contours, -1, cv::Scalar(0,0,255));
    //show(src, "Contours");

    int mark, A,B,C;
    mark = A = B = C = 0;

    std::vector<std::vector<cv::Point>> markContours;

    for (int i = 0; i < contours.size(); i++) {
        int k=i;
        int c=0;

        while(hierarchy[k][2] != -1) {
            k = hierarchy[k][2] ;
            c = c+1;
        }
        if(hierarchy[k][2] != -1)
            c = c+1;

        if (c >= 5) {
            markContours.push_back(contours[i]);
            //cv::drawContours(src, contours, i, cv::Scalar(0,255,0));
            if (mark == 0)		A = i;
            else if  (mark == 1)	B = i;		// i.e., A is already found, assign current contour to B
            else if  (mark == 2)	C = i;		// i.e., A and B are already found, assign current contour to C
            mark = mark + 1 ;
        }
    }

    //cv::drawContours(src, markContours, -1, cv::Scalar(255, 0, 0));
    //show(src, "Marks All");


    if (mark >= 3) {
        cv::drawContours(src, contours, A, cv::Scalar(255,0,0));
        cv::drawContours(src, contours, B, cv::Scalar(255,0,0));
        cv::drawContours(src, contours, C, cv::Scalar(255,0,0));
        show(src, "Marks");
    }

    return 0;
}