#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

/*
	 ласс дл€ поиска маркеров QR-кода
	на изображении.
*/
class QrDetector 
{

public:

	/*
		¬ыполн€ет поиск маркеров QR-кода.
		јргументы:
			image - входное изображение
			corners - выходной вектор, содержащий 4 вершины
			пр€моугольника, содержащий QR-код, отсортированные по стандарту
			расположени€ маркеров.
		¬озвращаемое значение: флаг наличи€ маркеров QR-кода на изображении.
	*/
	bool findQr(const cv::Mat &image, std::vector<cv::Point> &corners);

private:

	/* ¬ыполн€ет поиск маркеров */
	bool findQrMarks(const cv::Mat &image, std::vector<std::vector<cv::Point>> &markContours);

	/* ќпредел€ет вершины области QR-кода */
	void findCorners(const std::vector<std::vector<cv::Point>> &contours,
		std::vector<cv::Point> &corners);

};
