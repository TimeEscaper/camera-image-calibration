//
// Created by sibirsky on 11.07.18.
//

#include "../include/QrDetector.h"
#include "../include/Geometry.h"


bool QrDetector::findQr(const cv::Mat &image, std::vector<cv::Point> &corners) {
    std::vector<std::vector<cv::Point>> markContours;
    if (!findQrMarks(image, markContours))
        return false;

    findCorners(markContours, corners);

    return true;
}

bool QrDetector::findQrMarks(const cv::Mat &image, std::vector<std::vector<cv::Point>> &markContours) {
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

void QrDetector::findCorners(const std::vector<std::vector<cv::Point>> &contours,
                 std::vector<cv::Point> &corners) {
    if (contours.size() != 3)
        return;
    corners.clear();

    int cath, hypR, hypL;

    // Determine triangle vertices
    cv::Point centers[3];
    for (int i = 0; i < 3; i++) {
        centers[i] = geom::center(contours[i]);
    }
    int c1 = 0; int c2 = 1; int c3 = 2;
    double c1c2 = geom::distance(centers[c1], centers[c2]);
    double c1c3 = geom::distance(centers[c1], centers[c3]);
    double c2c3 = geom::distance(centers[c2], centers[c3]);
    if (c1c2 > c1c3 && c1c2 > c2c3) {
        cath = c3; hypR = c1; hypL = c2;
    } else if (c1c3 > c1c2 && c1c3 > c2c3) {
        cath = c2; hypR = c1; hypL = c3;
    } else {
        cath = c1; hypR = c2; hypL = c3;
    }
    if (centers[hypR].x < centers[hypL].x) {
        int buffer = hypR; hypR = hypL; hypL = buffer;
    }

    //Process cath mark
    double maxDist = 0;
    int cathMax = 0;
    for (int i = 0; i < contours[cath].size(); i++) {
        double dist = geom::distance(centers[hypR], centers[hypL], contours[cath][i]);
        if (dist > maxDist) {
            maxDist = dist; cathMax = i;
        }
    }

    //Process hypR mark
    double minDist = INFINITY;
    int hypRMin = 0;
    for (int i = 0; i < contours[hypR].size(); i++) {
        cv::Point point = contours[hypR][i];
        if (point.x > centers[hypR].x) {
            double dist = geom::distance(centers[hypR], centers[hypL], point);
            if (dist < minDist) {
                minDist = dist; hypRMin = i;
            }
        }
    }

    // Process hypL mark
    minDist = INFINITY;
    int hypLMin = 0;
    for (int i = 0; i < contours[hypL].size(); i++) {
        cv::Point point = contours[hypL][i];
        if (point.x < centers[hypL].x) {
            double dist = geom::distance(centers[hypR], centers[hypL], point);
            if (dist < minDist) {
                minDist = dist; hypLMin = i;
            }
        }
    }

    maxDist = 0;
    int hypRMax = 0;
    for (int i = 0; i < contours[hypR].size(); i++) {
        cv::Point point = contours[hypR][i];
        double diagDist = geom::distance(centers[hypR], centers[hypL], point);
        double sideDist = geom::distance(contours[cath][cathMax], contours[hypR][hypRMin], point);
        if (diagDist + sideDist > maxDist) {
            maxDist = diagDist + sideDist; hypRMax = i;
        }
    }

    maxDist = 0;
    int hypLMax = 0;
    for (int i = 0; i < contours[hypL].size(); i++) {
        cv::Point point = contours[hypL][i];
        double diagDist = geom::distance(centers[hypR], centers[hypL], point);
        double sideDist = geom::distance(contours[cath][cathMax], contours[hypL][hypLMin], point);
        if (diagDist + sideDist > maxDist) {
            maxDist = diagDist + sideDist; hypLMax = i;
        }
    }

    cv::Point2f fourth;
    geom::intersectionPoint(contours[hypR][hypRMin], contours[hypR][hypRMax],
                            contours[hypL][hypLMin], contours[hypL][hypLMax], fourth);

    cv::Point hypCenter = cv::Point((centers[hypL].x+centers[hypR].x)/2,
                                    (centers[hypL].y+centers[hypR].y)/2);
    corners.push_back(contours[cath][cathMax]);
    if (centers[cath].y <= hypCenter.y) {
        corners.push_back(contours[hypR][hypRMin]);
        corners.push_back(fourth);
        corners.push_back(contours[hypL][hypLMin]);
    } else {
        corners.push_back(contours[hypL][hypLMin]);
        corners.push_back(fourth);
        corners.push_back(contours[hypR][hypRMin]);
    }
}