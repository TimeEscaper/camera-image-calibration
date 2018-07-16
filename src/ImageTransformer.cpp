//
// Created by sibirsky on 11.07.18.
//

#include <opencv2/imgproc.hpp>
#include <opencv/cv.hpp>
#include "../include/ImageTransformer.h"
#include "../include/Geometry.h"

ImageTransformer::ImageTransformer() { }

ImageTransformer::ImageTransformer(const std::vector<cv::Point> &rectPoints) {
    //sortPoints(rectPoints, points);

    points = rectPoints;

    width = std::max(geom::distance(points[2], points[3]), geom::distance(points[1], points[0]));
    height = std::max(geom::distance(points[1], points[2]), geom::distance(points[0], points[3]));
    size = 300;

    /**
    cv::Point2f dest[4];
    dest[0] = cv::Point2f(0,0);
    dest[1] = cv::Point2f(size-1, 0);
    dest[2] = cv::Point2f(size-1, size-1);
    dest[3] = cv::Point2f(0, size-1);
     */

    std::vector<cv::Point> dest;
    dest.push_back(cv::Point(0, 0));
    dest.push_back(cv::Point(299, 0));
    dest.push_back(cv::Point(299, 299));
    dest.push_back(cv::Point(0, 299));

    transformMat = cv::findHomography(points, dest);

    //transformMat = cv::getPerspectiveTransform(src, dest);

}

ImageTransformer::ImageTransformer(ImageTransformer &other) {
    points = other.points;
    size = other.size;
    transformMat = other.transformMat;
}

ImageTransformer& ImageTransformer::operator=(const ImageTransformer &other) {
    if (&other == this)
        return *this;

    this->points = other.points;
    this->size = other.size;
    this->transformMat = other.transformMat;
}

void ImageTransformer::sortPoints(const std::vector<cv::Point> &points, std::vector<cv::Point> &sorted) {
    int maxY = 0;
    int minY = INFINITY;
    int minX = INFINITY;
    int maxX = 0;
    int maxXIndex, minXIndex, maxYIndex, minYIndex;
    for (int i = 0; i < points.size(); i++) {
        if (points[i].y >= maxY) {
            maxYIndex = i; maxY = points[i].y;
        }
        if (points[i].y <= minY) {
            minYIndex = i; minY = points[i].y;
        }
        if (points[i].x > maxX) {
            maxXIndex = i; maxX = points[i].x;
        }
        if (points[i].x < minX) {
            minXIndex = i; minX = points[i].x;
        }
    }

    if (points[minXIndex].y < points[maxXIndex].y) {
        sorted.push_back(points[minYIndex]);
        sorted.push_back(points[maxXIndex]);
        sorted.push_back(points[maxYIndex]);
        sorted.push_back(points[minXIndex]);
    } else {
        sorted.push_back(points[minXIndex]);
        sorted.push_back(points[minYIndex]);
        sorted.push_back(points[maxXIndex]);
        sorted.push_back(points[maxYIndex]);
    }

    /*
    cv::Point sortedPoints[4];
    int maxSum = points[0].x + points[0].y;
    int minSum = maxSum;
    int maxDiff = points[0].x - points[0].y;
    int minDiff = maxDiff;
    int minSumIndex = 0; int maxSumIndex = 0;
    int minDiffIndex = 0; int maxDiffIndex = 0;
    for (int i = 1; i < points.size(); i++) {
        int sum = points[i].x + points[i].y;
        if (sum > maxSum) {
            maxSum = sum; maxSumIndex = i;
        }
        else if (sum < minSum) {
            minSum = sum; minSumIndex = i;
        }
        int diff = points[i].x - points[i].y;
        if (diff > maxDiff) {
            maxDiff = diff; maxDiffIndex = i;
        } else if (diff < minDiff) {
            minDiff = diff; minDiffIndex = i;
        }
    }
    sortedPoints[0] = points[minSumIndex];
    sortedPoints[2] = points[maxSumIndex];
    sortedPoints[1] = points[minDiffIndex];
    sortedPoints[3] = points[maxDiffIndex];

    sorted.clear();
    for (int i = 0; i < 4; i++)
        sorted.push_back(sortedPoints[i]);
        */
}

void ImageTransformer::transform(const cv::Mat &image, cv::Mat &transformed) {
    cv::warpPerspective(image, transformed, transformMat, cv::Size(size, size));
}