#pragma once
#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

#define DEFAULT_SIZE 500

/*
	 ласс, предназначенный дл€
	выполнени€ преобразовани€ перспективы изображени€.
*/
class ImageTransformer 
{

public:
	ImageTransformer();
	ImageTransformer(const std::vector<cv::Point> &rectPoints);
	ImageTransformer(ImageTransformer &other);

	ImageTransformer& operator=(const ImageTransformer &other);

	/*
		¬ыполн€ет преобразование перспективы
		изображени€ на основе гомографии.
		јргументы:
			image - исходное изображение
			transformed - результат преобразовани€
	*/
	void transform(const cv::Mat &image, cv::Mat &transformed);

private:

	/*
		“очки, соответствующие отсортированным
		вершинам области с QR-маркерами
	*/
	std::vector<cv::Point> points;

	// ћатрица преобразовани€
	cv::Mat transformMat;

	// –азмер выходного изображен€
	int size = DEFAULT_SIZE;
};

