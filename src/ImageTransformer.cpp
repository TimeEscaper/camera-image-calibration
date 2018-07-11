//
// Created by sibirsky on 11.07.18.
//

#include <opencv2/imgproc.hpp>
#include "../include/ImageTransformer.h"
#include "../include/Geometry.h"

ImageTransformer::ImageTransformer(const std::vector<cv::Point> &rectPoints) {
    sortPoints(rectPoints, points);

    double width = std::max(geom::distance(points[2], points[3]), geom::distance(points[1], points[0]));
    double height = std::max(geom::distance(points[1], points[2]), geom::distance(points[0], points[3]));
    size = std::max(width, height);

    cv::Point2f src[4];
    for (int i = 0; i < 4; i++)
        src[i] = points[i];

    cv::Point2f dest[4];
    dest[0] = cv::Point2f(0,0);
    dest[1] = cv::Point2f(size-1, 0);
    dest[2] = cv::Point2f(size-1, size-1);
    dest[3] = cv::Point2f(0, size-1);

    transformMat = cv::getPerspectiveTransform(src, dest);
}

ImageTransformer::ImageTransformer(ImageTransformer &other) {
    points = other.points;
    size = other.size;
    transformMat = other.transformMat;
}

void ImageTransformer::sortPoints(const std::vector<cv::Point> &points, std::vector<cv::Point> &sorted) {
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
}

void ImageTransformer::transform(const cv::Mat &image, cv::Mat &transformed) {
    cv::warpPerspective(image, transformed, transformMat, cv::Size(size, size));
}