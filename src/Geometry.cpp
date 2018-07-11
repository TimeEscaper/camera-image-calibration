//
// Created by sibirsky on 11.07.18.
//

#include <opencv2/imgproc.hpp>
#include "../include/Geometry.h"

double geom::distance(cv::Point2f a, cv::Point2f b) {
    return cv::norm(a - b);
}

double geom::distance(cv::Point2f L, cv::Point2f M, cv::Point2f J) {
    double a,b,c,dist;
    a = -((M.y - L.y) / (M.x - L.x));
    b = 1.0;
    c = (((M.y - L.y) /(M.x - L.x)) * L.x) - L.y;

    dist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));

    return std::abs(dist);
}

cv::Point geom::center(std::vector<cv::Point> contour) {
    cv::Moments moments = cv::moments(contour, false);
    return cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
}

float geom::cross(cv::Point2f v1,cv::Point2f v2)
{
    return v1.x*v2.y - v1.y*v2.x;
}

bool geom::intersectionPoint(cv::Point2f a1, cv::Point2f a2, cv::Point2f b1, cv::Point2f b2,
                                   cv::Point2f& intersection)
{
    cv::Point2f p = a1;
    cv::Point2f q = b1;
    cv::Point2f r(a2-a1);
    cv::Point2f s(b2-b1);

    if(cross(r,s) == 0) {return false;}

    float t = cross(q-p,s)/cross(r,s);

    intersection = p + t*r;
    return true;
}