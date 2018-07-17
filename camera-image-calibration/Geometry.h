#pragma once
#include <opencv2/core/types.hpp>

/*
	Вспомогательные геометрические функции.
*/
namespace geom 
{
	// Расстояние между двумя точками
	double distance(cv::Point2f a, cv::Point2f b);

	// Расстояние между точкой и прямой
	double distance(cv::Point2f linePoint1, cv::Point2f linePoint2, cv::Point2f point);

	// Определитель 2 x 2
	float det(cv::Point2f v1, cv::Point2f v2);
	
	// Центр контура (центр масс)
	cv::Point center(std::vector<cv::Point> contour);

	// Определяет пересечение прямых
	bool intersectionPoint(cv::Point2f a1, cv::Point2f a2, cv::Point2f b1, cv::Point2f b2,
		cv::Point2f& intersection);
}

