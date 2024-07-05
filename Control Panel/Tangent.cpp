#include <ControlPanel.h>
#include <cmath>

using namespace std;
using namespace cv;


extern vector<Point2f> filtered_path;

Point2f& tangent(const size_t& i) {
	static Point2f T;
	static const size_t r = 20; // neighborhood radius, positive integer
	static size_t len = 0;

	static size_t iL = 0;
	static size_t iU = 0;
	static size_t j;
	static float hyp;

	static float den;

	len = filtered_path.size();


	if (i < r) {
		iL = 0;
		iU = 2 * r;
	}
	else if (len - (i + 1) < r) {
		iL = len - 2 * r - 1;
		iU = len - 1;
	}
	else {
		iL = i - r;
		iU = i + r;
	}

	T.x = 0.0;
	T.y = 0.0;


	for (j = iL; j < i; j++) {
		den = 1.0 / (i - j);
		T.x += den * (filtered_path[i].x - filtered_path[j].x);
		T.y += den * (filtered_path[i].y - filtered_path[j].y);
	}

	for (j = i + 1; j <= iU; j++) {
		den = 1.0 / (i - j);
		T.x += den * (filtered_path[i].x - filtered_path[j].x);
		T.y += den * (filtered_path[i].y - filtered_path[j].y);
	}

	hyp = 1.0 / hypot(T.x, T.y);

	T.x *= hyp;
	T.y *= hyp;


	return T;
}



vector<Point2f> filter(const vector<Point>& path) {

	const size_t r = 10;
	vector<Point2f> R;
	size_t n = path.size();
	float sum_x, sum_y;

	size_t i;

	for (i = 0; i < r; i++) {
		R.push_back(Point2f(path[i].x, path[i].y));
	}


	for (i = r; i < n - r; i++) {
		sum_x = 0.0;
		sum_y = 0.0;

		for (size_t j = i - r; j <= i + r; j++) {
			sum_x += path[j].x;
			sum_y += path[j].y;
		}

		sum_x /= (2 * r + 1);
		sum_y /= (2 * r + 1);

		R.push_back(Point2f(sum_x, sum_y));
	}


	for (i = n - r; i < n; i++) {
		R.push_back(Point2f(path[i].x, path[i].y));
	}

	return R;
}



vector<Point> discrete(vector<Point2f> path) {

	vector<Point> R;
	Point temp;

	for (Point2f P : path) {
		temp.x = (int)P.x;
		temp.y = (int)P.y;

		R.push_back(temp);
	}

	return R;
}