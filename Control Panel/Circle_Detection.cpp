#include <ControlPanel.h>

using namespace cv;
using namespace std;


char color_detector(const Vec3b& C) {

    static const Vec3b B = Vec3b(255, 0, 0);
    static const Vec3b G = Vec3b(0, 255, 0);
    static const Vec3b R = Vec3b(0, 0, 255);
    static float dB, dG, dR;

    dB = norm(C, B);
    dG = norm(C, G);
    dR = norm(C, R);

    static float min = 0;
    min = (dB <= dG) ? dB : dG;
    min = (min <= dR) ? min : dR;

    static char X = 0;

    if (min >= dB) {
        X = 0;
    }

    if (min >= dG) {
        X = 1;
    }

    if (min >= dR) {
        X = 2;
    }

    return X;

}



Mat& detector(const Mat& input) {

    static const int minSize = 10;
    static const float threshold1 = 10.0;
    static const float threshold2 = 50.0;
    static const float minAsp = 0.9;
    static const float maxAsp = 1.1;
    static const float confidence = 0.80;
    static const float PI = 3.141592;
    static const Size size = Size(3, 3);
    static const TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0);
    static Mat M(3, 2, CV_32F);
    static Mat BGR(3, 2, CV_32F);
    static const Mat N(1, 1, CV_32F);
    static Point P1, P2, P3;
    static Vec3b C1, C2, C3;
    static char color1, color2, color3;


    static Mat img;
    GaussianBlur(input, img, size, 0);

    static Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);


    Canny(gray, gray, threshold1, threshold2, 3, true);

    static vector<vector<Point>> contours;

    findContours(gray, contours, RETR_TREE, CHAIN_APPROX_NONE);

    //------------ stage 1
    vector<vector<Point>> stage1_contours;
    static Rect rect;

    for (vector<Point> c : contours) {
        rect = boundingRect(c);

        if (rect.width > minSize && rect.height > minSize) {
            stage1_contours.push_back(c);
        }
    }

    //------------ stage 2
    vector<vector<Point>> stage2_contours;
    static float asp;

    for (vector<Point> c : stage1_contours) {
        rect = boundingRect(c);
        asp = rect.width / (rect.height + 0.01);

        if (asp > minAsp && asp < maxAsp) {
            stage2_contours.push_back(c);
        }
    }

    //------------ stage 3
    vector<Point2f> data;
    static Point2f center;
    static float radius;
    static float area;
    static float EA;

    for (vector<Point> c : stage2_contours) {
        minEnclosingCircle(c, center, radius);
        area = contourArea(c);
        EA = PI * radius * radius;

        if (area > EA * confidence) {
            data.push_back(center);
        }
    }

    static vector<int> bestLabels;
    static Mat centers;

    if (data.size() < 3) {
        centers = N;
    }
    else if (data.size() == 3) {

        M.at<float>(0, 0) = data[0].x;
        M.at<float>(0, 1) = data[0].y;

        M.at<float>(1, 0) = data[1].x;
        M.at<float>(1, 1) = data[1].y;

        M.at<float>(2, 0) = data[2].x;
        M.at<float>(2, 1) = data[2].y;

        centers = M;
    }
    else {
        kmeans(data, 3, bestLabels, criteria, 10, KMEANS_RANDOM_CENTERS, centers);
    }

    if (centers.rows == 3) {
        P1.x = (int)centers.at<float>(0, 0);
        P1.y = (int)centers.at<float>(0, 1);

        P2.x = (int)centers.at<float>(1, 0);
        P2.y = (int)centers.at<float>(1, 1);

        P3.x = (int)centers.at<float>(2, 0);
        P3.y = (int)centers.at<float>(2, 1);

        C1 = img.at<Vec3b>(P1.y, P1.x);
        C2 = img.at<Vec3b>(P2.y, P2.x);
        C3 = img.at<Vec3b>(P3.y, P3.x);

        color1 = color_detector(C1);
        color2 = color_detector(C2);
        color3 = color_detector(C3);

        BGR.at<float>(color1, 0) = centers.at<float>(0, 0);
        BGR.at<float>(color1, 1) = centers.at<float>(0, 1);

        BGR.at<float>(color2, 0) = centers.at<float>(1, 0);
        BGR.at<float>(color2, 1) = centers.at<float>(1, 1);

        BGR.at<float>(color3, 0) = centers.at<float>(2, 0);
        BGR.at<float>(color3, 1) = centers.at<float>(2, 1);
    }




    return BGR;
}