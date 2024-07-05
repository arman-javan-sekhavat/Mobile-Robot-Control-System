#include "ControlPanel.h"
#include <QtWidgets/QApplication>


#include <QtWidgets/QApplication>
#include <QThread>
#include <QLabel>
#include <QPixmap>
#include <QString>
#include <QSlider>
#include <QPushButton>
#include <QLineEdit>
#include <QMenubar>
#include <QTextEdit>
#include <QTimer>
#include <QTcpSocket>
#include <QIcon>

#include <vector>
#include <string>
#include <cmath>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <winsock.h>


using namespace cv;
using namespace std;


const float rad = 57.2957795;


extern SOCKET serverSock = -1;
extern SOCKET clientSock = -1;


struct PHI {
    float phi_R;
    float phi_L;
};


struct PWM {
    short pwm_R;
    short pwm_L;
};



Point2f B, G, R;
Mat centers;

// rx1, ry1 , rx2, ry2 : mm
float rx1, ry1, Y1, Y2;
float rx2, ry2, T1, T2;
float irx2, iry2;

float theta;
float d_theta;

PHI* phi = nullptr;
PWM* pwm = nullptr;

Replay* obj = nullptr;


float hyp;


vector<Point2f> corners;
extern vector<Point2f> filtered_path = {};
vector<Point> path;
vector<vector<Point>> paths;

int width = 0;
int height = 0;
float W = 0.0;
float H = 0.0;
float scale = 0.0;
float inv_scale = 0.0;

bool drawing = false;
bool moving = false;
bool server = false;
bool connected = false;



QPushButton* CameraView = nullptr;
QPushButton* TransView = nullptr;
QPushButton* pServer = nullptr;
QPushButton* pMove = nullptr;
QTextEdit* text = nullptr;
QTimer* timer = nullptr;
QTimer* control_timer = nullptr;


QLabel* lvx = nullptr;
QLabel* lvy = nullptr;
QLabel* lvt = nullptr;

QLabel* dlvx = nullptr;
QLabel* dlvy = nullptr;
QLabel* dlvt = nullptr;

QLabel* connection = nullptr;
QLabel* server_stat = nullptr;


QLineEdit* LEKp = nullptr;
QLineEdit* LEKi = nullptr;
QLineEdit* LEKd = nullptr;

CONTROLLER* ctrl = nullptr;

QSlider* slider = nullptr;
short sliderValue = 0;

VideoCapture cap;

bool draw = false;

bool isAdjacent(Point P, Point Q) {
    return ((abs(P.x - Q.x) <= 1) && (abs(P.y - Q.y) <= 1));
}


vector<Point> Bresenham(int x1, int y1, int x2, int y2) {
    vector<Point> points;

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        points.push_back(cv::Point(x1, y1));
        if (x1 == x2 && y1 == y2) {
            break;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }

    return points;
}


vector<Point> modify(vector<Point> V) {
    vector<Point> modified;

    size_t n = V.size();
    Point P;
    Point Q;
    vector<Point> temp;

    for (size_t i = 0; i < n - 1; i++) {
        P = V[i];
        Q = V[i + 1];
        modified.push_back(P);

        if (!isAdjacent(P, Q)) {

            temp = Bresenham(P.x, P.y, Q.x, Q.y);
            modified.insert(modified.end(), temp.begin(), temp.end());
        }
    }

    return modified;
}



void DRAW_callback(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDBLCLK) {
        drawing = !drawing;
        text->append("x");

        if (drawing) {
            path.clear();
        }
        else {
            path = modify(path);
            filtered_path = filter(path);
            paths.clear();
            paths.push_back(discrete(filtered_path));
            draw = true;
        }
    }

    if (event == EVENT_MOUSEMOVE && drawing) {
        path.push_back(Point(x, y));
        text->append(QString::number(x) + ", " + QString::number(y));
    }
}


class Thread1 : public QThread {

public:

    bool show = false;

    void run() {

        Mat frame;
        namedWindow("Camera View");

        while (true) {

            cap.read(frame);


            imshow("Camera View", frame);
            waitKey(1);

            if (!show) {
                destroyWindow("Camera View");
                break;
            }

        }
    }
};
Thread1* Th1 = nullptr;


class Thread2 : public QThread {

public:

    Mat M;

    bool show = false;

    void run() {

        Mat frame;
        Mat T_frame;
        namedWindow("Transformed View");
        setMouseCallback("Transformed View", DRAW_callback);

        while (true) {

            cap.read(frame);
            warpPerspective(frame, T_frame, M, Size((int)W, (int)H));

            centers = detector(T_frame);

            B.x = centers.at<float>(0, 0);
            B.y = centers.at<float>(0, 1);

            G.x = centers.at<float>(1, 0);
            G.y = centers.at<float>(1, 1);

            R.x = centers.at<float>(2, 0);
            R.y = centers.at<float>(2, 1);

            rx1 = (B.x + G.x + R.x) * inv_scale;
            ry1 = (B.y + G.y + R.y) * inv_scale;

            Y1 = 2 * R.x - G.x - B.x;
            Y2 = 2 * R.y - G.y - B.y;

            hyp = 1.0 / hypot(Y1, Y2);
            Y1 *= hyp;
            Y2 *= hyp;

            if (fabs(Y1) > 0.0001) {
                theta = rad * atan2f(Y2, Y1);
            }

            lvx->setText(QString::number(rx1*0.1));
            lvy->setText(QString::number(ry1*0.1));
            lvt->setText(QString::number(theta));


            if (draw) {
                drawContours(T_frame, paths, 0, Scalar(0, 0, 255), 1);
            }

            line(T_frame, Point(irx2 - T1*50, iry2 - T2 * 50), Point(irx2 + T1 * 50, iry2 + T2 * 50), Scalar(0, 255, 0), 1);
            line(T_frame, Point(irx2 - T2 * 50, iry2 + T1 * 50), Point(irx2 + T2 * 50, iry2 - T1 * 50), Scalar(0, 255, 0), 1);

            line(T_frame, Point(R.x, R.y), Point(G.x, G.y), Scalar(0, 0, 0), 1);
            line(T_frame, Point(G.x, G.y), Point(B.x, B.y), Scalar(0, 0, 0), 1);
            line(T_frame, Point(R.x, R.y), Point(B.x, B.y), Scalar(0, 0, 0), 1);

            imshow("Transformed View", T_frame);
            waitKey(1);

            if (!show) {
                destroyWindow("Transformed View");
                break;
            }

        }
    }
};
Thread2* Th2 = nullptr;


class Thread3 : public QThread {

public:

    void run() {

        if (startServer() == 0) {
            server_stat->setText("Server status:     Started");

            clientSock = accept(serverSock, NULL, NULL);

            if (clientSock != -1) {
                connection->setText("Connected");
                

                for (int i = 0; i < 15; i++) {
                    obj->initState();
                }

                connected = true;


            }
        }


        // send and receive
        
    }
};

Thread3* Th3 = nullptr;



void Replay::myFunction() {

    if ((i < filtered_path.size()) && moving) {
        P = filtered_path[i];

        irx2 = (filtered_path[i].x);
        iry2 = (filtered_path[i].y);

        rx2 = (filtered_path[i].x)/scale;
        ry2 = (filtered_path[i].y)/scale;
        T1 = tangent(i).x;
        T2 = tangent(i).y;

        if (fabs(T1) > 0.0001) {
            d_theta = rad * atan2f(T2, T1);
        }

        dlvx->setText(QString::number(rx2*0.1));
        dlvy->setText(QString::number(ry2*0.1));
        dlvt->setText(QString::number(d_theta));

        i++;
    }
}


void Replay::initState() {

    if (i < filtered_path.size()) {
        P = filtered_path[i];

        irx2 = (filtered_path[i].x);
        iry2 = (filtered_path[i].y);

        rx2 = (filtered_path[i].x) / scale;
        ry2 = (filtered_path[i].y) / scale;
        T1 = tangent(i).x;
        T2 = tangent(i).y;

        if (fabs(T1) > 0.0001) {
            d_theta = rad * atan2f(T2, T1);
        }

        dlvx->setText(QString::number(rx2*0.1));
        dlvy->setText(QString::number(ry2*0.1));
        dlvt->setText(QString::number(d_theta));

        i++;
    }

}


float HardTanh(float x) {
    static float r = 0;

    if ((x >= -255) && (x <= +255)) {
        r = x;
    }
    else if (x < -255) {
        r = -255;
    }
    else {
        r = +255;
    }

    return r;

}


void CONTROLLER::controller() {

    if (!connected) {
        return;
    }

    if (!started) {
        e_x = rx2 - rx1;
        e_y = ry2 - ry1;
        e_t = Y1 * T2 - Y2 * T1;

        started = true;
    }

    old_e_x = e_x;
    old_e_y = e_y;
    old_e_t = e_t;

    e_x = rx2 - rx1;
    e_y = ry2 - ry1;
    e_t = Y1 * T2 - Y2 * T1;

    ie_x += e_x * timestep;
    ie_y += e_y * timestep;
    ie_t += e_t * timestep;

    de_x = (e_x - old_e_x) / timestep;
    de_y = (e_y - old_e_y) / timestep;
    de_t = (e_t - old_e_t) / timestep;


    fe_x = Kp * e_x + Ki * ie_x + Kd * de_x;
    fe_y = Kp * e_y + Ki * ie_y + Kd * de_y;
    fe_t = Kp * e_t + Ki * ie_t + Kd * de_t;


    E1 = Y1 * fe_x + Y2 * fe_y;
    E2 = S * fe_t;

    pwm->pwm_R = round(HardTanh(E1 + E2));
    pwm->pwm_L = round(HardTanh(E1 - E2));

    /*if (fabs(pwm->pwm_R) < 100) {
        pwm->pwm_R = (pwm->pwm_R > 0) ? +100 : -100;
    }

    if (fabs(pwm->pwm_L) < 100) {
        pwm->pwm_L = (pwm->pwm_L > 0) ? +100 : -100;
    }*/


    //pwm->pwm_R = sliderValue;
    //pwm->pwm_L = sliderValue;


    send(clientSock, (const char*)pwm, sizeof(PWM), 0);


}



void computeM(void) {

    vector<Point2f> dst;
    W = scale * width;
    H = scale * height;

    text->append(QString::number(W));
    text->append(QString::number(H));

    dst.push_back(Point2f(0, 0));
    dst.push_back(Point2f(0, H));
    dst.push_back(Point2f(W, H));
    dst.push_back(Point2f(W, 0));

    Th2->M = getPerspectiveTransform(corners, dst, INTER_LANCZOS4);
}


void CV_callback(int event, int x, int y, int flags, void* userdata) {
    if ((corners.size() < 4) && (event == EVENT_LBUTTONDOWN)) {
        corners.push_back(Point2f(x, y));
        text->append(QString::number(x) + ", " + QString::number(y) + '\n');
    }
}




void Actions::MarkCorners(void) {
    corners.clear();
    setMouseCallback("Camera View", CV_callback);
}

void Actions::ShowCameraView(void) {

    Th1->show = !Th1->show;

    CameraView->setText(Th1->show ? "Hide camera view" : "Show camera view");

    if (Th1->show) {
        Th1->run();
    }
}


void Actions::ShowTransView(void) {

    Th2->show = !Th2->show;

    TransView->setText(Th2->show ? "Hide transformed view" : "Show transformed view");

    if (Th2->show) {
        computeM();
        Th2->run();
    }

}


void Actions::ScaleChanged(const QString& text) {
    scale = text.toFloat();
    inv_scale = 1.0 / (3.0 * scale);
}

void Actions::WidthChanged(const QString& text) {
    width = text.toInt();
}

void Actions::HeightChanged(const QString& text) {
    height = text.toInt();
}

void Actions::disconnect(void) {

}


void Actions::Move(void) {

    moving = !moving;

    if (moving) {
        pMove->setText("Stop");
        timer->start(5);
    }
    else {
        pMove->setText("Move");
        timer->stop();
    }

}


void Actions::Server(void) {

    server = !server;

    if (server) {
        pServer->setText("Stop the server");
        Th3->start();
        control_timer->start(100);

    }
    else {
        pServer->setText("Start the server");

        if (stopServer() == 0) {
            server_stat->setText("Server status:     Stopped");
            connection->setText("Not connected");
            connected = false;
        }
    }

}


void Actions::Apply(void) {
    ctrl->Kp = LEKp->text().toFloat();
    ctrl->Ki = LEKi->text().toFloat();
    ctrl->Kd = LEKd->text().toFloat();
}


void Actions::Reset(void) {

    timer->stop();
    pMove->setText("Move");
    moving = false;

    obj->i = 0;

    for (int i = 0; i < 15; i++) {
        obj->initState();
    }

    ctrl->ie_x = 0.0;
    ctrl->ie_y = 0.0;
    ctrl->ie_t = 0.0;
}


void Actions::Test(void) {
    sliderValue = slider->value();
    text->setText(QString::number(sliderValue));
}

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);

    //------------------------------------  Main Window
    ControlPanel w;
    w.setFixedSize(728, 450);
    w.setWindowTitle("Control Panel");


    Actions actions;

    QLabel Lwidth(&w);
    Lwidth.setGeometry(50, 10, 80, 30);
    Lwidth.setText("Width (mm):");

    QLineEdit LEwidth(&w);
    LEwidth.setGeometry(130, 15, 70, 20);

    QLabel Lheight(&w);
    Lheight.setGeometry(50, 40, 80, 30);
    Lheight.setText("Height (mm):");

    QLineEdit LEheight(&w);
    LEheight.setGeometry(130, 45, 70, 20);

    QPushButton MarkCorners(&w);
    MarkCorners.setGeometry(50, 80, 150, 30);
    MarkCorners.setText("Mark the corners");


    QLabel Scale(&w);
    Scale.setGeometry(50, 130, 80, 30);
    Scale.setText("Scale (px/mm):");

    QLineEdit LEscale(&w);
    LEscale.setGeometry(140, 130, 60, 30);

    pServer = new QPushButton(&w);
    pServer->setGeometry(50, 180, 150, 30);
    pServer->setText("Start the server");

    pMove = new QPushButton(&w);
    pMove->setGeometry(50, 230, 150, 30);
    pMove->setText("Move");


    CameraView = new QPushButton(&w);
    CameraView->setGeometry(50, 280, 150, 30);
    CameraView->setText("Show camera view");

    TransView = new QPushButton(&w);
    TransView->setGeometry(50, 330, 150, 30);
    TransView->setText("Show transformed view");

    QLabel conn_stat(&w);
    conn_stat.setGeometry(50, 380, 120, 30);
    conn_stat.setText("Connection status:");

    connection = new QLabel(&w);
    connection->setGeometry(170, 380, 150, 30);
    connection->setText("Not connected");

    server_stat = new QLabel(&w);
    server_stat->setGeometry(50, 400, 200, 30);
    server_stat->setText("Server status:     Stopped");

    QLabel about(&w);
    about.setGeometry(500, 400, 200, 30);
    about.setText("By Arman Javan Sekhavat");

    QLabel coordinates(&w);
    coordinates.setGeometry(330, 10, 150, 30);
    coordinates.setText("Robot's real coordinates");

    QLabel lx(&w);
    lx.setGeometry(330, 40, 50, 30);
    lx.setText("x =");

    lvx = new QLabel(&w);
    lvx->setGeometry(380, 40, 50, 30);
    lvx->setText("0.0");

    QLabel ly(&w);
    ly.setGeometry(330, 70, 50, 30);
    ly.setText("y =");

    lvy = new QLabel(&w);
    lvy->setGeometry(380, 70, 50, 30);
    lvy->setText("0.0");

    QLabel ltheta(&w);
    ltheta.setGeometry(330, 100, 50, 30);
    ltheta.setText("theta =");

    lvt = new QLabel(&w);
    lvt->setGeometry(380, 100, 50, 30);
    lvt->setText("0.0");

    QLabel unitx(&w);
    unitx.setGeometry(470, 40, 50, 30);
    unitx.setText("cm");

    QLabel unity(&w);
    unity.setGeometry(470, 70, 50, 30);
    unity.setText("cm");

    QLabel unitt(&w);
    unitt.setGeometry(470, 100, 50, 30);
    unitt.setText("deg");


    QLabel des_coordinates(&w);
    des_coordinates.setGeometry(330, 200, 150, 30);
    des_coordinates.setText("Robot's desired coordinates");


    QLabel dlx(&w);
    dlx.setGeometry(330, 230, 50, 30);
    dlx.setText("x =");

    dlvx = new QLabel(&w);
    dlvx->setGeometry(380, 230, 50, 30);
    dlvx->setText("0.0");

    QLabel dly(&w);
    dly.setGeometry(330, 260, 50, 30);
    dly.setText("y =");

    dlvy = new QLabel(&w);
    dlvy->setGeometry(380, 260, 50, 30);
    dlvy->setText("0.0");

    QLabel dltheta(&w);
    dltheta.setGeometry(330, 290, 50, 30);
    dltheta.setText("theta =");

    dlvt = new QLabel(&w);
    dlvt->setGeometry(380, 290, 50, 30);
    dlvt->setText("0.0");

    QLabel dunitx(&w);
    dunitx.setGeometry(470, 230, 50, 30);
    dunitx.setText("cm");

    QLabel dunity(&w);
    dunity.setGeometry(470, 260, 50, 30);
    dunity.setText("cm");

    QLabel dunitt(&w);
    dunitt.setGeometry(470, 290, 50, 30);
    dunitt.setText("deg");



    QLabel LKp(&w);
    LKp.setGeometry(550, 10, 30, 30);
    LKp.setText("Kp =");

    QLabel LKi(&w);
    LKi.setGeometry(550, 50, 30, 30);
    LKi.setText("Ki  =");

    QLabel LKd(&w);
    LKd.setGeometry(550, 90, 30, 30);
    LKd.setText("Kd =");


    LEKp = new QLineEdit(&w);
    LEKp->setGeometry(580, 15, 70, 20);

    LEKi = new QLineEdit(&w);
    LEKi->setGeometry(580, 55, 70, 20);

    LEKd = new QLineEdit(&w);
    LEKd->setGeometry(580, 95, 70, 20);

    QPushButton apply(&w);
    apply.setGeometry(550, 130, 100, 62);
    apply.setText("Apply");

    QPushButton reset(&w);
    reset.setGeometry(550, 230, 100, 62);
    reset.setText("Reset");



    obj = new Replay;
    timer = new QTimer;

    ctrl = new CONTROLLER;
    control_timer = new QTimer;

    slider = new QSlider;
    slider->setMinimum(0);
    slider->setMaximum(255);
    slider->show();

    


    QObject::connect(timer, SIGNAL(timeout()), obj, SLOT(myFunction()));
    QObject::connect(control_timer, SIGNAL(timeout()), ctrl, SLOT(controller()));


    QObject::connect(&MarkCorners, SIGNAL(clicked()), &actions, SLOT(MarkCorners()));
    QObject::connect(CameraView, SIGNAL(clicked()), &actions, SLOT(ShowCameraView()));
    QObject::connect(TransView, SIGNAL(clicked()), &actions, SLOT(ShowTransView()));
    QObject::connect(&LEscale, SIGNAL(textEdited(const QString&)), &actions, SLOT(ScaleChanged(const QString&)));
    QObject::connect(&LEwidth, SIGNAL(textEdited(const QString&)), &actions, SLOT(WidthChanged(const QString&)));
    QObject::connect(&LEheight, SIGNAL(textEdited(const QString&)), &actions, SLOT(HeightChanged(const QString&)));
    QObject::connect(pMove, SIGNAL(clicked()), &actions, SLOT(Move()));
    QObject::connect(pServer, SIGNAL(clicked()), &actions, SLOT(Server()));
    QObject::connect(&apply, SIGNAL(clicked()), &actions, SLOT(Apply()));
    QObject::connect(&reset, SIGNAL(clicked()), &actions, SLOT(Reset()));


    phi = new PHI;
    pwm = new PWM;

    Th1 = new Thread1;
    Th2 = new Thread2;
    Th3 = new Thread3;
    


    text = new QTextEdit;
    text->show();

    QObject::connect(slider, SIGNAL(valueChanged(int)), &actions, SLOT(Test()));

    cap.open(0);

    QIcon icon("icon.ico");
    w.setWindowIcon(icon);

    w.show();



    return a.exec();
}

