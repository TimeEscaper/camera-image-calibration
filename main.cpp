#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#define TEST_IMAGE "/home/sibirsky/calibration_images/t55LVeq3esM.jpg"

void show(cv::Mat image, const char* name);
bool findQrMarks(const cv::Mat &image, std::vector<std::vector<cv::Point>> &markContours);
void findCorners(const std::vector<std::vector<cv::Point>> &contours,
                 std::vector<cv::Point> &corners);
double distance(cv::Point2f a, cv::Point2f b);
double distance(cv::Point2f linePoint1, cv::Point2f linePoint2, cv::Point2f point);
cv::Point center(std::vector<cv::Point> contour);
void getLineParameters(cv::Point2f point1, cv::Point2f point2, double &slope, double &b);

void show(cv::Mat image, const char* name) {
    cv::namedWindow(name);
    cv::imshow(name, image);
    cv::waitKey();
    cv::destroyWindow(name);
}

double distance(cv::Point2f a, cv::Point2f b) {
    return cv::norm(a - b);
}

double distance(cv::Point2f L, cv::Point2f M, cv::Point2f J) {
    double a,b,c,pdist;

    a = -((M.y - L.y) / (M.x - L.x));
    b = 1.0;
    c = (((M.y - L.y) /(M.x - L.x)) * L.x) - L.y;

    // Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

    pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));

    if (pdist < 0)
        pdist = -pdist;

    return pdist;
}

cv::Point center(std::vector<cv::Point> contour) {
    cv::Moments moments = cv::moments(contour, false);
    return cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
}

void getLineParameters(cv::Point2f point1, cv::Point2f point2, double &slope, double &b) {
    slope = (point2.y - point1.y) / (point2.x - point1.x);
    b - point1.y - slope * point1.x;
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

void findCorners(const std::vector<std::vector<cv::Point>> &contours,
                 std::vector<cv::Point> &corners) {
    if (contours.size() != 3)
        return;
    corners.clear();

    int cath, hypR, hypL;

    // Determine triangle vertices
    cv::Point centers[3];
    for (int i = 0; i < 3; i++) {
        centers[i] = center(contours[i]);
    }
    int c1 = 0; int c2 = 1; int c3 = 2;
    double c1c2 = distance(centers[c1], centers[c2]);
    double c1c3 = distance(centers[c1], centers[c3]);
    double c2c3 = distance(centers[c2], centers[c3]);
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
    double cathMax = 0;
    for (int i = 0; i < contours[cath].size(); i++) {
        double dist = distance(centers[hypR], centers[hypL], contours[cath][i]);
        if (dist > maxDist) {
            maxDist = dist; cathMax = i;
        }
    }
    corners.push_back(contours[cath][cathMax]);

    //Process hypR mark
    double minDist = INFINITY;
    double hypRMin = 0;
    for (int i = 0; i < contours[hypR].size(); i++) {
        cv::Point point = contours[hypR][i];
        if (point.x > centers[hypR].x) {
            double dist = distance(centers[hypR], centers[hypL], point);
            if (dist < minDist) {
                minDist = dist; hypRMin = i;
            }
        }
    }
    corners.push_back(contours[hypR][hypRMin]);

    minDist = INFINITY;
    double hypLMin = 0;
    for (int i = 0; i < contours[hypL].size(); i++) {
        cv::Point point = contours[hypL][i];
        if (point.x < centers[hypL].x) {
            double dist = distance(centers[hypR], centers[hypL], point);
            if (dist < minDist) {
                minDist = dist; hypLMin = i;
            }
        }
    }
    corners.push_back(contours[hypL][hypLMin]);
}

void computeFourthCorner(std::vector<cv::Point> &corners) {

    float slope1 = ((float)(corners[0].y - corners[1].y)) / ((float)(corners[0].x - corners[1].x));
    float slope2 = ((float)(corners[0].y - corners[2].y)) / ((float)(corners[0].x - corners[2].x));

    float b1 = corners[2].y - slope1 * corners[2].x;
    float b2 = corners[1].y - slope2 * corners[1].x;

    float delta = -slope1 + slope2;
    float delta1 = b1 - b2;
    float delta2 = -slope1*b2 + slope2*b1;

    cv::Point2f point(delta1/delta, delta2/delta);
    corners.push_back(point);

}

int main() {
    cv::Mat src = cv::imread(TEST_IMAGE);
    show(src, "Source");

    std::vector<std::vector<cv::Point>> markContours;
    if (!findQrMarks(src, markContours))
        return 0;

    cv::drawContours(src, markContours, -1, cv::Scalar(255, 0, 0));
    show(src, "QR marks");

    std::vector<cv::Point> corners;
    findCorners(markContours, corners);
    //computeFourthCorner(corners);
    for (int i = 0; i < corners.size(); i++) {
        cv::circle(src, corners[i], 5, cv::Scalar(0,0,255));
    }

    /**
    cv::RotatedRect rect = cv::minAreaRect(corners);

    cv::Point2f rect_points[4]; rect.points( rect_points );
    for( int j = 0; j < 4; j++ )
        line( src, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,255,0), 1, 8 );
**/
    //cv::line(src, corners[0], corners[1], cv::Scalar(0,255,0));
    //cv::line(src, corners[0], corners[2], cv::Scalar(0,255,0));
    //cv::line(src, corners[3], corners[1], cv::Scalar(0,255,0));
    //cv::line(src, corners[3], corners[2], cv::Scalar(0,255,0));

    show(src, "Corners");

    return 0;
}