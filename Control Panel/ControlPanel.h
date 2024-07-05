#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_ControlPanel.h"
#include <opencv2/opencv.hpp>
#include <vector>

class ControlPanel : public QMainWindow
{
    Q_OBJECT

public:
    ControlPanel(QWidget *parent = nullptr);
    ~ControlPanel();

private:
    Ui::ControlPanelClass ui;
};



class Actions : public QObject {
    Q_OBJECT

private slots:

    void MarkCorners(void);
    void ShowCameraView(void);
    void ShowTransView(void);
    void ScaleChanged(const QString&);
    void WidthChanged(const QString&);
    void HeightChanged(const QString&);
    void disconnect(void);
    void Move(void);
    void Server(void);
    void Apply(void);
    void Reset(void);
    void Test(void);
};


class Replay : public QObject {
    Q_OBJECT

public:
    size_t i = 0;
    cv::Point P;

public:
    void initState();

public slots:
    void myFunction();
};


class CONTROLLER : public QObject {
    Q_OBJECT

public:
    float Kp = 1;
    float Ki = 0;
    float Kd = 1;


public:
    // S = s/2 (mm)
    float S = 97.5;

    float e_x;
    float e_y;
    float e_t;

    float old_e_x;
    float old_e_y;
    float old_e_t;

    float ie_x = 0.0;
    float ie_y = 0.0;
    float ie_t = 0.0;

    float de_x;
    float de_y;
    float de_t;

    float fe_x;
    float fe_y;
    float fe_t;

    float E1;
    float E2;

    float timestep = 0.1;

    bool started = false;

public slots:
    void controller();
};





cv::Mat& detector(const cv::Mat&);
cv::Point2f& tangent(const size_t&);
std::vector<cv::Point2f> filter(const std::vector<cv::Point>&);
std::vector<cv::Point> discrete(std::vector<cv::Point2f>);


int startServer(void);
int stopServer(void);