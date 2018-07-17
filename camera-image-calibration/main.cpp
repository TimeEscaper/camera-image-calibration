#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "QrDetector.h"
#include "ImageTransformer.h"

int main() 
{
	cv::VideoCapture capture(0);
	if (!capture.isOpened()) 
	{
		std::cout << "Unable to open camera!" << std::endl;
		std::getchar();
		return -1;
	}
	std::cout << "Camera opened" << std::endl <<
		"Press \'q\' at active image window to exit application" << std::endl;

	cv::Mat frame;
	cv::Mat calibrated;

	QrDetector qrDetector;
	ImageTransformer transformer;
	std::vector<cv::Point> corners;

	bool found = false;

	cv::namedWindow("Source", CV_WINDOW_AUTOSIZE);

	while (true)
	{
		capture >> frame;
		cv::imshow("Source", frame);

		if ((char)cv::waitKey(10) == 'q')
			break;
		
		if (found) 
		{
			transformer.transform(frame, calibrated);
			cv::imshow("Calibrated", calibrated);
		}
		else if (qrDetector.findQr(frame, corners))
		{
			std::cout << "QR marks detected" << std::endl;
			transformer = ImageTransformer(corners);
			found = true;
		}
		
	}

	cv::destroyAllWindows();

	return 0;
}