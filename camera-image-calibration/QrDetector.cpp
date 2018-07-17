#include "QrDetector.h"
#include "Geometry.h"


bool QrDetector::findQr(const cv::Mat &image, std::vector<cv::Point> &corners) 
{
	std::vector<std::vector<cv::Point>> markContours;
	if (!findQrMarks(image, markContours))
		return false;

	findCorners(markContours, corners);

	return true;
}

bool QrDetector::findQrMarks(const cv::Mat &image, std::vector<std::vector<cv::Point>> &markContours) 
{
	// Предобработка: сглаживание
	cv::Mat blured;
	cv::blur(image, blured, cv::Size(3, 3));

	// Предобработка: конвертация в градации серого
	cv::Mat gray;
	cv::cvtColor(blured, gray, CV_BGR2GRAY);

	// Предобработка: адаптивное пороговое преобразование
	cv::Mat bw;
	cv::adaptiveThreshold(gray, bw, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 51, 0);

	// Применение детектора границ
	cv::Mat edges;
	cv::Canny(bw, edges, 50, 150, 5);

	// Поиск контуров на изображении и их иерархии
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	// Три маркера QR-кода
	int markCount, mark1, mark2, mark3;
	markCount = mark1 = mark2 = mark3 = 0;

	for (int i = 0; i < contours.size(); i++) 
	{
		int child = i;
		int children = 0;

		// Определяем количество вложенных контуров
		// (дочерний контур находится на третьем месте в иерархии)
		while (hierarchy[child][2] != -1) 
		{
			child = hierarchy[child][2];
			children++;
		}
		if (hierarchy[child][2] != -1)
			children++;

		// С учетом внтуренних и внешних границ
		if (children >= 5) 
		{
			switch (markCount) 
			{
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

void QrDetector::findCorners(const std::vector<std::vector<cv::Point>> &contours,
	std::vector<cv::Point> &corners) 
{
	if (contours.size() != 3)
		return;
	corners.clear();
	
	/* 
		Определяем порядок трех маркеров:
		cath - на пересечении катетов треугольника маркеров
		hypR - вершина на гипотенузе, расположенная правее по оси X
		hypL - вершина на гипотенузе, располоденная левее по оси X
	*/
	int cath, hypR, hypL;

	// Определяем центры контуров
	cv::Point centers[3];
	for (int i = 0; i < 3; i++) 
	{
		centers[i] = geom::center(contours[i]);
	}

	// На основе неравенства треугольника определяем типы вершин
	int c1 = 0; int c2 = 1; int c3 = 2;
	double c1c2 = geom::distance(centers[c1], centers[c2]);
	double c1c3 = geom::distance(centers[c1], centers[c3]);
	double c2c3 = geom::distance(centers[c2], centers[c3]);
	if (c1c2 > c1c3 && c1c2 > c2c3) {
		cath = c3; hypR = c1; hypL = c2;
	}
	else if (c1c3 > c1c2 && c1c3 > c2c3) {
		cath = c2; hypR = c1; hypL = c3;
	}
	else {
		cath = c1; hypR = c2; hypL = c3;
	}
	if (centers[hypR].x < centers[hypL].x) {
		int buffer = hypR; hypR = hypL; hypL = buffer;
	}

	/*
		Определяем угол области у маркера cath -
		он находится дальше всех от диагонали.
	*/
	double maxDist = 0;
	int cathMax = 0;
	for (int i = 0; i < contours[cath].size(); i++) 
	{
		double dist = geom::distance(centers[hypR], centers[hypL], contours[cath][i]);
		if (dist > maxDist) {
			maxDist = dist; cathMax = i;
		}
	}

	/*
		Определяем угол области у маркера hypR -
		он находится правее всех остальных и ближе
		всех к диагонали.
	*/
	double minDist = INFINITY;
	int hypRMin = 0;
	for (int i = 0; i < contours[hypR].size(); i++) 
	{
		cv::Point point = contours[hypR][i];
		if (point.x > centers[hypR].x) {
			double dist = geom::distance(centers[hypR], centers[hypL], point);
			if (dist < minDist) {
				minDist = dist; hypRMin = i;
			}
		}
	}

	/*
		Определяем угол области у маркера hypL -
		он находится левее всех остальных и ближе
		всех к диагонали.
	*/
	minDist = INFINITY;
	int hypLMin = 0;
	for (int i = 0; i < contours[hypL].size(); i++) 
	{
		cv::Point point = contours[hypL][i];
		if (point.x < centers[hypL].x) {
			double dist = geom::distance(centers[hypR], centers[hypL], point);
			if (dist < minDist) {
				minDist = dist; hypLMin = i;
			}
		}
	}

	/*
		Находим угол маркера hypR, лежащий на
		одной прямой с углом области этого мракера
		и оставшимся четвертым углом области - эта
		точка максимально удалена от диагонали и от
		стороны области.
	*/
	maxDist = 0;
	int hypRMax = 0;
	for (int i = 0; i < contours[hypR].size(); i++) 
	{
		cv::Point point = contours[hypR][i];
		double diagDist = geom::distance(centers[hypR], centers[hypL], point);
		double sideDist = geom::distance(contours[cath][cathMax], contours[hypR][hypRMin], point);
		if (diagDist + sideDist > maxDist) {
			maxDist = diagDist + sideDist; hypRMax = i;
		}
	}

	/*
		Находим угол маркера hypL, лежащий на
		одной прямой с углом области этого мракера
		и оставшимся четвертым углом области - эта
		точка максимально удалена от диагонали и от
		стороны области.
	*/
	maxDist = 0;
	int hypLMax = 0;
	for (int i = 0; i < contours[hypL].size(); i++) 
	{
		cv::Point point = contours[hypL][i];
		double diagDist = geom::distance(centers[hypR], centers[hypL], point);
		double sideDist = geom::distance(contours[cath][cathMax], contours[hypL][hypLMin], point);
		if (diagDist + sideDist > maxDist) {
			maxDist = diagDist + sideDist; hypLMax = i;
		}
	}

	/*
		Определяем четвертый угол как пересечение
		прямых, проходящих через стороны маркеров
	*/
	cv::Point2f fourth;
	geom::intersectionPoint(contours[hypR][hypRMin], contours[hypR][hypRMax],
		contours[hypL][hypLMin], contours[hypL][hypLMax], fourth);

	// Сортируем полученные углы в порядке стандартного следования маркеров QR-кода
	cv::Point hypCenter = cv::Point((centers[hypL].x + centers[hypR].x) / 2,
		(centers[hypL].y + centers[hypR].y) / 2);
	corners.push_back(contours[cath][cathMax]);
	if (centers[cath].y <= hypCenter.y) {
		corners.push_back(contours[hypR][hypRMin]);
		corners.push_back(fourth);
		corners.push_back(contours[hypL][hypLMin]);
	}
	else {
		corners.push_back(contours[hypL][hypLMin]);
		corners.push_back(fourth);
		corners.push_back(contours[hypR][hypRMin]);
	}
}