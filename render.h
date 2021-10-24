#ifndef RENDER_H
#define RENDER_H

#include "GL/freeglut.h"
#include "GL/gl.h"
#include "GL/glu.h"
#include <vector>
#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
//#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/aruco/charuco.hpp>
#include <mutex>
#include <unistd.h>
#include <QThread>
#include <thread>
//#include <psmoveapi/psmove.h>
#include <psmove.h>


class GLRender
{
    static cv::Mat wheel;
    static GLuint colorWheel;
    static bool found,found2;
    static int notFoundCount,notFoundCount2;
    static int ticks,ticks2;
    static cv::KalmanFilter kf,kf2;
    static cv::Mat state,state2,meas,meas2;
    static PSMove*controller;
    static std::vector<std::vector<cv::Point3d>> strokes,strokesU,tmp;
    static std::vector<cv::Point3d> verticies,strokeColor,strokeColorU;
    static cv::Point3d color;
    static bool colorMode,shapeMode;
    static std::vector<int> strokeType,strokeLayer,layers,strokeTypeU,strokeLayerU;
    static int tool,toolbelt,layer;
    static cv::aruco::DetectorParameters params;
    static cv::Ptr<cv::aruco::DetectorParameters> parametersPtr;
    static std::vector<cv::Vec3d> rvec1,tvec1,rvec2,tvec2,rvec3,tvec3;
    static bool isArucoCalibrated,isArucoCalibrated3;
    static const int B = 1000;
    static const int G = 1000;
    static const int FRAME_AVERAGE = 30;
    bool static isCalibrated;
    cv::Mat static camMat1, camMat2, coeff1,coeff2,camMat3,coeff3;
    std::vector<cv::Point2i> static pointStack1;
    std::vector<cv::Point2i> static pointStack2;
    std::vector<cv::Point2i> static pointStack2debug;
    int static erodeIterations;
    int static dilateIterations;
    int static erodeBrushSize;
    int static dilateBrushSize;
    int static w1,w2;
    int static h1,h2;
    bool static win1valid, win2valid,win3valid;
    cv::VideoCapture static cap1,cap2,cap3;
    GLuint static texture;
    GLuint static texture2;
    cv::Mat static frame1, frame2,frame3;
    cv::Mat static local2;
    std::mutex static mut1, mut2,mut3;
    std::vector<std::thread*> static threads;
    std::thread static *glutLoop;
    int static window,win2,win3,win4;
    bool static win1debugmode, win2debugmode;
    std::vector<cv::Mat> static rvecs1,tvecs1,rvecs2,tvecs2;

    std::vector<cv::Mat> calibPics1,calibPics2,calibPics3;

    void static lights(int i,float dx,float dy, float dz,float px, float py, float pz);
    void static orthogonalStart();
    void static orthogonalEnd();
    void static background(GLuint t);
    GLuint static LoadTexture( cv::Mat m );
    void init();
    void static draw();
    void static renderCylinder(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions,GLUquadricObj *quadric);
    void static renderCylinder_convenient(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions);
    void static draw2();
    void static draw3();
    void static reshape(GLsizei width, GLsizei height);
    void static reshape2(GLsizei width, GLsizei height);
    void static vidCap1();
    void static vidCap2();
    void static vidCap3();
    //void keyboardFunkc(unsigned char key,int x,int y);
    void kill();
    int vidCapAsynch1();
    int vidCapAsynch2();
    int vidCapAsynch3();

public:

    QString ip1, ip2,ip3;
    bool isSuccess;
    static bool isDrawing;
    bool static debug2;
    static float a,b,c,size;
    void static loadCalib();
    void createKnownBoardPositions(cv::Size boardSize, float edgeLengt,std::vector<cv::Point3f>& corners);
    void getChessboardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f>>& foundCorners, bool showResults);
    int calibrate1();
    int calibrate2();
    int calibrate3();
    int calibrate();
    GLRender();
    void win1o();
    void win3o();
    void win1c();
    void win3c();
    void win1d(int arg);
    int ipChanged();
    void startGlutLoop();

    void getPictures1();//pobierz FRAME_AVERAGE klatek do kalibracji
    void getPictures2();
    void getPictures3();
    void dumpPictures();
    ~GLRender();
};

#endif // RENDER_H
