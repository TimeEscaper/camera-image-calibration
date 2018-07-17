#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

/*
	����� ��� ������ �������� QR-����
	�� �����������.
*/
class QrDetector 
{

public:

	/*
		��������� ����� �������� QR-����.
		���������:
			image - ������� �����������
			corners - �������� ������, ���������� 4 �������
			��������������, ���������� QR-���, ��������������� �� ���������
			������������ ��������.
		������������ ��������: ���� ������� �������� QR-���� �� �����������.
	*/
	bool findQr(const cv::Mat &image, std::vector<cv::Point> &corners);

private:

	/* ��������� ����� �������� */
	bool findQrMarks(const cv::Mat &image, std::vector<std::vector<cv::Point>> &markContours);

	/* ���������� ������� ������� QR-���� */
	void findCorners(const std::vector<std::vector<cv::Point>> &contours,
		std::vector<cv::Point> &corners);

};
