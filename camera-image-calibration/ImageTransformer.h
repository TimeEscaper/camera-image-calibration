#pragma once
#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

#define DEFAULT_SIZE 500

/*
	�����, ��������������� ���
	���������� �������������� ����������� �����������.
*/
class ImageTransformer 
{

public:
	ImageTransformer();
	ImageTransformer(const std::vector<cv::Point> &rectPoints);
	ImageTransformer(ImageTransformer &other);

	ImageTransformer& operator=(const ImageTransformer &other);

	/*
		��������� �������������� �����������
		����������� �� ������ ����������.
		���������:
			image - �������� �����������
			transformed - ��������� ��������������
	*/
	void transform(const cv::Mat &image, cv::Mat &transformed);

private:

	/*
		�����, ��������������� ���������������
		�������� ������� � QR-���������
	*/
	std::vector<cv::Point> points;

	// ������� ��������������
	cv::Mat transformMat;

	// ������ ��������� ����������
	int size = DEFAULT_SIZE;
};

