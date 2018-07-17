#include <opencv2/imgproc.hpp>
#include <opencv/cv.hpp>
#include "ImageTransformer.h"
#include "Geometry.h"

ImageTransformer::ImageTransformer() { }

ImageTransformer::ImageTransformer(const std::vector<cv::Point> &rectPoints) 
{
	points = rectPoints;

	/*
		—тавим в соотетствие вершинам области
		с QR-маркерами врешины квадрата, расположенного
		ровно ("вид сверху").
	*/
	std::vector<cv::Point> dest;
	int max = size - 1;
	dest.push_back(cv::Point(0, 0));
	dest.push_back(cv::Point(max, 0));
	dest.push_back(cv::Point(max, max));
	dest.push_back(cv::Point(0, max));

	// ќпредел€ем преобразование гомографии
	transformMat = cv::findHomography(points, dest);
}

ImageTransformer::ImageTransformer(ImageTransformer &other) 
{
	points = other.points;
	size = other.size;
	transformMat = other.transformMat;
}

ImageTransformer& ImageTransformer::operator=(const ImageTransformer &other) 
{
	if (&other == this)
		return *this;

	this->points = other.points;
	this->size = other.size;
	this->transformMat = other.transformMat;
}


void ImageTransformer::transform(const cv::Mat &image, cv::Mat &transformed) 
{
	cv::warpPerspective(image, transformed, transformMat, cv::Size(size, size));
}