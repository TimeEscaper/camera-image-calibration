#pragma once
#include <opencv2/core/types.hpp>

/*
	��������������� �������������� �������.
*/
namespace geom 
{
	// ���������� ����� ����� �������
	double distance(cv::Point2f a, cv::Point2f b);

	// ���������� ����� ������ � ������
	double distance(cv::Point2f linePoint1, cv::Point2f linePoint2, cv::Point2f point);

	// ������������ 2 x 2
	float det(cv::Point2f v1, cv::Point2f v2);
	
	// ����� ������� (����� ����)
	cv::Point center(std::vector<cv::Point> contour);

	// ���������� ����������� ������
	bool intersectionPoint(cv::Point2f a1, cv::Point2f a2, cv::Point2f b1, cv::Point2f b2,
		cv::Point2f& intersection);
}

