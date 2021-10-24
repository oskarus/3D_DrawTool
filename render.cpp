#include "render.h"
#include "GL/freeglut.h"
#include "GL/gl.h"
#include "GL/glu.h"
#include <vector>
#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/aruco/charuco.hpp>
#include <mutex>
#include <unistd.h>
#include <QThread>
#include <QDebug>
#include <cmath>
#include <math.h>
#include <fstream>
#include <algorithm>
#include <numeric>
using namespace std;
using namespace cv;
Mat GLRender::wheel;
GLuint GLRender::colorWheel;
Mat GLRender::state,GLRender::state2,GLRender::meas,GLRender::meas2;
KalmanFilter GLRender::kf,GLRender::kf2;
int GLRender::notFoundCount,GLRender::notFoundCount2,GLRender::ticks,GLRender::ticks2;
bool GLRender::isArucoCalibrated,GLRender::isArucoCalibrated3,GLRender::found,GLRender::found2;
PSMove* GLRender::controller;
aruco::DetectorParameters GLRender::params;
Ptr<aruco::DetectorParameters> GLRender::parametersPtr;
vector<Mat> GLRender::rvecs1,GLRender::tvecs1,GLRender::rvecs2,GLRender::tvecs2;
const int GLRender::FRAME_AVERAGE,GLRender::G,GLRender::B;
const float calibrationSquareDimention = 0.02465; //meters 0.023
const float arucoDims = 0.1001;//10,01cm
const Size chessboardDimensions = Size(6, 9);
Mat GLRender::camMat1, GLRender::camMat2, GLRender::coeff1, GLRender::coeff2, GLRender::coeff3, GLRender::camMat3;
vector<Point2i> GLRender::pointStack1;
vector<Point2i> GLRender::pointStack2;
vector<Point2i> GLRender::pointStack2debug;
int GLRender::erodeIterations;
int GLRender::dilateIterations;
int GLRender::erodeBrushSize;
int GLRender::dilateBrushSize;
int GLRender::w1;
int GLRender::h1;
int GLRender::window;
int GLRender::win2;
int GLRender::win3;
bool GLRender::win1valid;
bool GLRender::win3valid,GLRender::win2valid;
bool GLRender::isCalibrated;
VideoCapture GLRender::cap1,GLRender::cap2,GLRender::cap3;
GLuint GLRender::texture;
GLuint GLRender::texture2;
Mat GLRender::frame1, GLRender::frame2, GLRender::frame3;
mutex GLRender::mut1, GLRender::mut2,GLRender::mut3;
vector<thread*> GLRender::threads;
thread* GLRender::glutLoop;
bool GLRender::win1debugmode,GLRender::win2debugmode,GLRender::debug2;
cv::Mat GLRender::local2;
float GLRender::a,GLRender::b,GLRender::c,GLRender::size;
vector<vector<Point3d>> GLRender::strokes,GLRender::strokesU,GLRender::tmp;
vector<Point3d> GLRender::verticies,GLRender::strokeColor,GLRender::strokeColorU;
Point3d GLRender::color;
bool GLRender::colorMode,GLRender::shapeMode;
vector<int> GLRender::strokeType,GLRender::strokeLayer,GLRender::layers,GLRender::strokeTypeU,GLRender::strokeLayerU;
int GLRender::tool,GLRender::toolbelt,GLRender::layer;
vector<Vec3d> GLRender::rvec1,GLRender::tvec1,GLRender::rvec2,GLRender::tvec2,GLRender::rvec3,GLRender::tvec3;
bool GLRender::isDrawing;

void GLRender::renderCylinder(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions,GLUquadricObj *quadric)
{
    float vx = x2-x1;
    float vy = y2-y1;
    float vz = z2-z1;

    //handle the degenerate case of z1 == z2 with an approximation
    if(vz == 0)
    vz = .0001;

    float v = sqrt( vx*vx + vy*vy + vz*vz );
    float ax = 57.2957795*acos( vz/v );
    if ( vz < 0.0 )
    ax = -ax;
    float rx = -vy*vz;
    float ry = vx*vz;
    glPushMatrix();

    //draw the cylinder body
    glTranslatef( x1,y1,z1 );
    glRotatef(ax, rx, ry, 0.0);
    if(z2<z1)
        glRotatef(180,1,0,0);
    gluQuadricOrientation(quadric,GLU_OUTSIDE);
    gluCylinder(quadric, radius, 0, v, subdivisions, 1);
    //gluDisk(quadric,0,radius,subdivisions,1);
    glPopMatrix();
    }

void GLRender::renderCylinder_convenient(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions)
    {
    //the same quadric can be re-used for drawing many cylinders
    GLUquadricObj *quadric=gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);
    renderCylinder(x1,y1,z1,x2,y2,z2,radius,subdivisions,quadric);
    gluDeleteQuadric(quadric);
}

void GLRender::lights(int i,float dx,float dy, float dz,float px, float py, float pz)
{
    if(i > 0)
    {
        glShadeModel(GL_SMOOTH);
        glEnable(GL_LIGHTING);
        glEnable( GL_LIGHT0 );
        // Create light components
        GLfloat ambientLight[] = { 0.6, 0.6, 0.6, 1.0 };
        GLfloat diffuseLight[] = { 1.2, 1.2, 1.2, 1.0 };
        GLfloat specularLight[] = { 1.6, 1.6, 1.6, 1.0 };
        GLfloat position[] = { px,py, pz, 1};
        GLfloat qa[] = {0};
        GLfloat la[] = {0};
        GLfloat ca[] = {1};
        GLfloat se[] = {32};
        GLfloat sc[] = {90};
        GLfloat sd[] = {dx,dy,dz};

        // Assign created components to GL_LIGHT0
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
        glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
        glLightfv(GL_LIGHT0, GL_POSITION, position);
        glLightfv(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, qa);
        glLightfv(GL_LIGHT0, GL_LINEAR_ATTENUATION, la);
        glLightfv(GL_LIGHT0, GL_CONSTANT_ATTENUATION, ca);
        glLightfv(GL_LIGHT0, GL_SPOT_EXPONENT, se);
        glLightfv(GL_LIGHT0, GL_SPOT_CUTOFF, sc);
        glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, sd);
    }
    else
    {
        glDisable(GL_LIGHTING);
    }
}

void GLRender::orthogonalStart()
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(-w1/2, w1/2, -h1/2, h1/2);
    glMatrixMode(GL_MODELVIEW);
}

void GLRender::orthogonalEnd()
{
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void GLRender::background(GLuint t)
{
    glDepthMask(GL_FALSE);
    glBindTexture( GL_TEXTURE_2D, t );

    orthogonalStart();

    // texture width/height
    //const int iw = 240;
    //const int ih = 320;

    const int iw = w1;
    const int ih = h1;

    glPushMatrix();
    glLoadIdentity();
    glTranslatef( -iw/2, -ih/2, 0 );
    glBegin(GL_QUADS);
    glTexCoord2i(0,0); glVertex2i(0, 0);
    glTexCoord2i(1,0); glVertex2i(iw, 0);
    glTexCoord2i(1,1); glVertex2i(iw, ih);
    glTexCoord2i(0,1); glVertex2i(0, ih);
    glEnd();
    glPopMatrix();

    //orthogonalEnd();
    glDepthMask(GL_TRUE);
}

GLuint GLRender::LoadTexture(Mat m)
{

    GLuint texture;

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


    if(m.channels() > 3)//Alpha channel
        glTexImage2D(GL_TEXTURE_2D,     // Type of texture
            0,                 // Pyramid level (for mip-mapping) - 0 is the top level
            GL_RGBA,            // Internal colour format to convert to
            m.cols,          // Image width  i.e. 640 for Kinect in standard mode
            m.rows,          // Image height i.e. 480 for Kinect in standard mode
            0,                 // Border width in pixels (can either be 1 or 0)
            GL_BGRA, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
            GL_UNSIGNED_BYTE,  // Image data type
            m.ptr());
    else//no Alpha channel
        glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                     0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                     GL_RGB,            // Internal colour format to convert to
                     m.cols,          // Image width  i.e. 640 for Kinect in standard mode
                     m.rows,          // Image height i.e. 480 for Kinect in standard mode
                     0,                 // Border width in pixels (can either be 1 or 0)
                     GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                     GL_UNSIGNED_BYTE,  // Image data type
                     m.ptr());        // The actual image data itself

    return texture;
}

void GLRender::init()
{
    glClearColor(0,0,0,1);
    glClearDepth(1);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void GLRender::draw()
{
    if(!win1valid){
        usleep(10000);
        return;
    }
    psmove_set_leds(controller,0,0,5);//100 w jasnym swietle 5 w normalnym
    psmove_update_leds(controller);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    Mat frame1local,frame2local;
    Mat calc;
    Mat tex;

    if(isArucoCalibrated == false)
    {
        tvec1.clear();
        tvec2.clear();
        rvec1.clear();
        rvec2.clear();
        Mat local1,local2;
        mut1.lock();
        frame1.copyTo(local1);
        mut1.unlock();
        mut2.lock();
        frame2.copyTo(local2);
        mut2.unlock();

        if(local1.empty() || local2.empty())
        {
            glutPostRedisplay();
            return;
        }
        flip(local1,local1,1);
        flip(local2,local2,1);
        vector<int> markerIDs;
        markerIDs.push_back(0);
        Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
        vector<vector<Point2f>> markercorners1,rej1;
        aruco::detectMarkers(local1,markerDictionary,markercorners1,markerIDs,parametersPtr,rej1);//,camMat1,coeff1);
        aruco::estimatePoseSingleMarkers(markercorners1,arucoDims,camMat1,coeff1,rvec1,tvec1);
        vector<vector<Point2f>> markercorners2,rej2;
        aruco::detectMarkers(local2,markerDictionary,markercorners2,markerIDs,parametersPtr,rej2);//,camMat2,coeff2);
        aruco::estimatePoseSingleMarkers(markercorners2,arucoDims,camMat2,coeff2,rvec2,tvec2);
        if(rvec1.size()>0){
            qDebug()<<1;
            aruco::drawAxis(local1,camMat1,coeff1,rvec1[0],tvec1[0],0.1f);
        }
        if(rvec2.size()>0){
            qDebug()<<2;
            aruco::drawAxis(local2,camMat2,coeff2,rvec2[0],tvec2[0],0.1f);
        }
        if(rvec1.size()>0&&rvec2.size()>0)
        {
            isArucoCalibrated = true;
            qDebug() << "aruco found!";
            glutPostRedisplay();
            return;
        }
        glDeleteTextures(1,&texture);
        Mat concat;
        hconcat(local1,local2,concat);
        texture  = LoadTexture(concat);
        glEnable( GL_TEXTURE_2D );
        background(texture);
        glDisable(GL_TEXTURE_2D);
        glutSwapBuffers();
        glutPostRedisplay();
        return;
    }

    mut1.lock();
    frame1.copyTo(frame1local);
    mut1.unlock();

    Point2d point1;

    Mat blur;
    cv::GaussianBlur(frame1local, blur, cv::Size(5, 5), 3.0, 3.0);
    // <<<<< Noise smoothing
    // >>>>> HSV conversion
    cv::Mat frmHsv;
    cv::cvtColor(blur, frmHsv, COLOR_BGR2HSV);
    // <<<<< HSV conversion
    // >>>>> Color Thresholding
    cv::Mat rangeRes = cv::Mat::zeros(frame1local.size(), CV_8UC1);
    cv::inRange(frmHsv, cv::Scalar(200 / 2, 100, 80),
        cv::Scalar(300 / 2, 255, 255), rangeRes);
    // <<<<< Color Thresholding
    // >>>>> Improving the result
    cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
    // <<<<< Improving the result

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int largest_area=0;
    int largest_contour_index=0;
    Rect bounding_rect;
    findContours(rangeRes, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    // <<<<< Filtering

    vector<vector<cv::Point> > balls;
    vector<cv::Rect> ballsBox;

    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Rect bBox;
        bBox = cv::boundingRect(contours[i]);
        float ratio = (float) bBox.width / (float) bBox.height;
        if (ratio > 1.0f)
            ratio = 1.0f / ratio;
        // Searching for a bBox almost square
        if (ratio > 0.75 && bBox.area() >= 400)
        {
            balls.push_back(contours[i]);
            ballsBox.push_back(bBox);
        }
    }
    // <<<<< Filtering
    // >>>>> Detection result
    /*for (size_t i = 0; i < balls.size(); i++)
    {
        cv::drawContours(frame1local, balls, i, CV_RGB(20,150,20), 2);
        cv::rectangle(frame1local, ballsBox[i], CV_RGB(0,255,0), 2);
        cv::Point center;
        center.x = ballsBox[i].x + ballsBox[i].width / 2;
        center.y = ballsBox[i].y + ballsBox[i].height / 2;
        cv::circle(frame1local, center, 2, CV_RGB(20,150,20), -1);
        stringstream sstr;
        sstr << "(" << center.x << "," << center.y << ")";
        cv::putText(frame1local, sstr.str(),
        cv::Point(center.x + 3, center.y - 3),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
    }*/
    // <<<<< Detection result

    double precTick = ticks;
    ticks = (double) cv::getTickCount();
    double dT = (ticks - precTick) / cv::getTickFrequency();
    if (found)
    {
        // >>>> Matrix A
        kf.transitionMatrix.at<float>(2) = dT;
        kf.transitionMatrix.at<float>(9) = dT;
        // <<<< Matrix A=
        state = kf.predict();
        cv::Rect predRect;
        predRect.width = state.at<float>(4);
        predRect.height = state.at<float>(5);
        predRect.x = state.at<float>(0) - predRect.width / 2;
        predRect.y = state.at<float>(1) - predRect.height / 2;
        cv::Point center;
        center.x = state.at<float>(0);
        center.y = state.at<float>(1);
        //cv::circle(frame1local, center, 2, CV_RGB(255,0,0), -1);
        //cv::rectangle(frame1local, predRect, CV_RGB(255,0,0), 2);
        point1.x = (double)center.x;
        point1.y = (double)center.y;

    }
    // >>>>> Kalman Update
    if (balls.size() == 0)
    {
        notFoundCount++;
        if( notFoundCount >= 100 )
        {
              found = false;
        }
    }
    else
    {
        notFoundCount = 0;

        meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
        meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
        meas.at<float>(2) = (float)ballsBox[0].width;
        meas.at<float>(3) = (float)ballsBox[0].height;
        if (!found) // First detection!
        {
            // >>>> Initialization
            kf.errorCovPre.at<float>(0) = 1; // px
            kf.errorCovPre.at<float>(7) = 1; // px
            kf.errorCovPre.at<float>(14) = 1;
            kf.errorCovPre.at<float>(21) = 1;
            kf.errorCovPre.at<float>(28) = 1; // px
            kf.errorCovPre.at<float>(35) = 1; // px
            state.at<float>(0) = meas.at<float>(0);
            state.at<float>(1) = meas.at<float>(1);
            state.at<float>(2) = 0;
            state.at<float>(3) = 0;
            state.at<float>(4) = meas.at<float>(2);
            state.at<float>(5) = meas.at<float>(3);
            // <<<< Initialization
            kf.statePost = state;
            found = true;
        }
        else
            kf.correct(meas); // Kalman Correction
    }
    // <<<<< Kalman Update

    stringstream sstr;
    sstr<< "toolbelt: " << toolbelt <<" ,tool: " << tool << " ,layer: " << layer;
    cv::putText(frame1local, sstr.str(),
    cv::Point(5, 10),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(20,150,20), 0.7,0,true);

    glDeleteTextures(1,&texture);
    texture  = LoadTexture(frame1local);

    contours.clear();
    hierarchy.clear();
    largest_area=0;
    largest_contour_index=0;
    balls.clear();
    ballsBox.clear();

    if(!win2valid)
        {
            mut2.lock();
            frame2local = frame2;
            mut2.unlock();
        }
        else
            frame2local = local2;

    //cam2 detect!!

    Point2d point2;

    cv::GaussianBlur(frame2local, blur, cv::Size(5, 5), 3.0, 3.0);
    // <<<<< Noise smoothing
    // >>>>> HSV conversion
    cv::cvtColor(blur, frmHsv, COLOR_BGR2HSV);
    // <<<<< HSV conversion
    // >>>>> Color Thresholding
    rangeRes = cv::Mat::zeros(frame2local.size(), CV_8UC1);
        cv::inRange(frmHsv, cv::Scalar(200 / 2, 100, 80),
        cv::Scalar(300 / 2, 255, 255), rangeRes);
    // <<<<< Color Thresholding
    // >>>>> Improving the result
    cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
    // <<<<< Improving the result

    findContours(rangeRes, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    // <<<<< Filtering
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Rect bBox;
        bBox = cv::boundingRect(contours[i]);
        float ratio = (float) bBox.width / (float) bBox.height;
        if (ratio > 1.0f)
            ratio = 1.0f / ratio;
        // Searching for a bBox almost square
        if (ratio > 0.75 && bBox.area() >= 400)
        {
            balls.push_back(contours[i]);
            ballsBox.push_back(bBox);
        }
    }
    // <<<<< Filtering
    double precTick2 = ticks2;
    ticks2 = (double) cv::getTickCount();
    double dT2 = (ticks2 - precTick2) / cv::getTickFrequency();
    if (found2)
    {
        // >>>> Matrix A
        kf2.transitionMatrix.at<float>(2) = dT2;
        kf2.transitionMatrix.at<float>(9) = dT2;
        // <<<< Matrix A=
        state2 = kf2.predict();
        cv::Rect predRect;
        predRect.width = state2.at<float>(4);
        predRect.height = state2.at<float>(5);
        predRect.x = state2.at<float>(0) - predRect.width / 2;
        predRect.y = state2.at<float>(1) - predRect.height / 2;
        cv::Point center;
        center.x = state2.at<float>(0);
        center.y = state2.at<float>(1);
        //cv::circle(res, center, 2, CV_RGB(255,0,0), -1);
        //cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
        point2.x = (double)center.x;
        point2.y = (double)center.y;

    }

    // >>>>> Kalman Update
    if (balls.size() == 0)
    {
        notFoundCount2++;
        if( notFoundCount2 >= 100 )
        {
              found2 = false;
        }
    }
    else
    {
        notFoundCount2 = 0;

        meas2.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
        meas2.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
        meas2.at<float>(2) = (float)ballsBox[0].width;
        meas2.at<float>(3) = (float)ballsBox[0].height;
        if (!found2) // First detection!
        {
            // >>>> Initialization
            kf2.errorCovPre.at<float>(0) = 1; // px
            kf2.errorCovPre.at<float>(7) = 1; // px
            kf2.errorCovPre.at<float>(14) = 1;
            kf2.errorCovPre.at<float>(21) = 1;
            kf2.errorCovPre.at<float>(28) = 1; // px
            kf2.errorCovPre.at<float>(35) = 1; // px
            state2.at<float>(0) = meas2.at<float>(0);
            state2.at<float>(1) = meas2.at<float>(1);
            state2.at<float>(2) = 0;
            state2.at<float>(3) = 0;
            state2.at<float>(4) = meas2.at<float>(2);
            state2.at<float>(5) = meas2.at<float>(3);
            // <<<< Initialization
            kf2.statePost = state2;
            found2 = true;
        }
        else
            kf2.correct(meas2); // Kalman Correction
    }
    // <<<<< Kalman Update

    Mat rotation,translation;

    if(isCalibrated)
    {
        double x=0,y=0,z=0;
        Point3d raay1,raay2;
        //obliczenia x y z bloba swiatla w 3d coordynatach swiata tytaj
        Mat rmat1,rmat2;
        Rodrigues(rvec1[0],rmat1);
        Rodrigues(rvec2[0],rmat2);
        Mat t1,t2;
        t1.push_back(tvec1[0][0]);
        t1.push_back(tvec1[0][1]);
        t1.push_back(tvec1[0][2]);
        Mat z1(1,3,t1.type());
        z1.col(0).at<double>(0) = 0;
        z1.col(0).at<double>(1) = 0;
        z1.col(0).at<double>(2) = 0;
        z1 = z1.t();
        t2.push_back(tvec2[0][0]);
        t2.push_back(tvec2[0][1]);
        t2.push_back(tvec2[0][2]);
        vector<double> l4 = {0,0,0,1};
        Mat T1,R1,T2;
        hconcat(rmat1,t1,T1);
        hconcat(rmat2,t2,T2);
        hconcat(rmat1,z1,R1);
        R1.push_back(Mat(l4).t());
        T1.push_back(Mat(l4).t());
        T2.push_back(Mat(l4).t());
        T1 = T1.inv();
        T2 = T2.inv();
        R1 = R1.inv();

        Mat cam2coords,cam1coords;
        Mat zero; zero.push_back(l4);
        cam2coords = T2 * zero;//przesuniecie
        cam1coords = T1 * zero;
        //Point3d camPoint1(0,0,0);
        Point3d camPoint2(cam2coords.col(0).at<double>(0)/cam2coords.col(0).at<double>(3),cam2coords.col(0).at<double>(1)/cam2coords.col(0).at<double>(3),cam2coords.col(0).at<double>(2)/cam2coords.col(0).at<double>(3));
        Point3d camPoint1(cam1coords.col(0).at<double>(0)/cam1coords.col(0).at<double>(3),cam1coords.col(0).at<double>(1)/cam1coords.col(0).at<double>(3),cam1coords.col(0).at<double>(2)/cam1coords.col(0).at<double>(3));

        if(found && found2){
            //obliczanie promieni kierunkowych
            vector<Point2d> imagePoints1,imagePoints2;
            //imagePoints1.push_back(Point2d(average1.x+frame1local.cols/2,average1.y+frame1local.rows/2));
            //imagePoints2.push_back(Point2d(average2.x+frame2local.cols/2,average2.y+frame2local.rows/2));
            imagePoints1.push_back(Point2d(frame1local.cols-point1.x,point1.y));
            imagePoints2.push_back(Point2d(frame2local.cols-point2.x,point2.y));
            //imagePoints1.push_back(Point2d(frame1local.rows-average1.y,average1.x));
            //imagePoints2.push_back(Point2d(frame2local.rows-average2.y,average2.x));
            vector<Point2d> Normalized1,Normalized2;
            cv::undistortPoints(imagePoints1,Normalized1,camMat1,coeff1);//p1 jest dla przesuniecia p2 tez
            cv::undistortPoints(imagePoints2,Normalized2,camMat2,coeff2);//,noArray(),PMat2);//presunac ray
            vector<Point3d> NormalizedHomogeneous1,NormalizedHomogeneous2;
            convertPointsToHomogeneous(Normalized1,NormalizedHomogeneous1);
            convertPointsToHomogeneous(Normalized2,NormalizedHomogeneous2);
            Mat NormalizedHomogeneousMatrix1=cv::Mat(NormalizedHomogeneous1);
            Mat NormalizedHomogeneousMatrix2=cv::Mat(NormalizedHomogeneous2);
            NormalizedHomogeneousMatrix1=NormalizedHomogeneousMatrix1.reshape(1).t();
            NormalizedHomogeneousMatrix2=NormalizedHomogeneousMatrix2.reshape(1).t();

            NormalizedHomogeneousMatrix1.push_back(Mat(vector<double> {1}).t());
            NormalizedHomogeneousMatrix1 = T1 * NormalizedHomogeneousMatrix1;
            NormalizedHomogeneousMatrix1 = NormalizedHomogeneousMatrix1/NormalizedHomogeneousMatrix1.at<double>(3,0);
            NormalizedHomogeneousMatrix2.push_back(Mat(vector<double> {1}).t());
            NormalizedHomogeneousMatrix2 = T2 * NormalizedHomogeneousMatrix2;
            NormalizedHomogeneousMatrix2 = NormalizedHomogeneousMatrix2/NormalizedHomogeneousMatrix2.at<double>(3,0);
            raay1 = Point3d(NormalizedHomogeneousMatrix1.at<double>(0,0),NormalizedHomogeneousMatrix1.at<double>(1,0),NormalizedHomogeneousMatrix1.at<double>(2,0));
            raay2 = Point3d(NormalizedHomogeneousMatrix2.at<double>(0,0),NormalizedHomogeneousMatrix2.at<double>(1,0),NormalizedHomogeneousMatrix2.at<double>(2,0));
            raay1 = raay1 / sqrt((raay1.x*raay1.x)+(raay1.y*raay1.y)+(raay1.z*raay1.z));
            raay2 = raay2 / sqrt((raay2.x*raay2.x)+(raay2.y*raay2.y)+(raay2.z*raay2.z));

            //szukanie punktu
            Point3d ray1 = raay1 - camPoint1, ray2 = raay2 - camPoint2;
            ray1 = ray1 / sqrt((ray1.x*ray1.x)+(ray1.y*ray1.y)+(ray1.z*ray1.z));
            ray2 = ray2 / sqrt((ray2.x*ray2.x)+(ray2.y*ray2.y)+(ray2.z*ray2.z));

            //qDebug() << "ray1: (" << ray1.x << "," << ray1.y << "," << ray1.z << ") ray2: (" << ray2.x << "," << ray2.y << "," <<  ray2.z << ")";
            //qDebug()<< "cam1: (" << camPoint1.x << "," <<  camPoint1.y << "," <<  camPoint1.z << ") cam2: (" << camPoint2.x << "," <<  camPoint2.y << "," <<  camPoint2.z << ")";

            double dataA[9] = {2-(ray1.x*ray1.x)-(ray2.x*ray2.x), -ray1.x*ray1.y-ray2.x*ray2.y, -ray1.x*ray1.z-ray2.x*ray2.z,
                              -ray1.x*ray1.y-ray2.x*ray2.y, 2-(ray1.y*ray1.y)-(ray2.y*ray2.y), -ray1.y*ray1.z-ray2.y*ray2.z,
                              -ray1.x*ray1.z-ray2.x*ray2.z, -ray1.y*ray1.z-ray2.y*ray2.z, 2-(ray1.z*ray1.z)-(ray2.z*ray2.z)};

            Mat A = Mat(3, 3, CV_64F, &dataA);
            double dataB[3] = {(1-(ray1.x*ray1.x))*camPoint1.x-ray1.x*ray1.y*camPoint1.y-ray1.x*ray1.z*camPoint1.z +
                               (1-(ray2.x*ray2.x))*camPoint2.x-ray2.x*ray2.y*camPoint2.y-ray2.x*ray2.z*camPoint2.z,
                               -ray1.x*ray1.y*camPoint1.x+(1-(ray1.y*ray1.y))*camPoint1.y-ray1.y*ray1.z*camPoint1.z
                               -ray2.x*ray2.y*camPoint2.x+(1-(ray2.y*ray2.y))*camPoint2.y-ray2.y*ray2.z*camPoint2.z,
                               -ray1.x*ray1.z*camPoint1.x-ray1.y*ray1.z*camPoint1.y+(1-(ray1.z*ray1.z))*camPoint1.z
                               -ray2.x*ray2.z*camPoint2.x-ray2.y*ray2.z*camPoint2.y+(1-(ray2.z*ray2.z))*camPoint2.z};
            Mat B = Mat(3, 1, CV_64F, &dataB);
            Mat punkt = A.inv()*B;
            Point3d punkt3d(punkt.col(0).at<double>(0),punkt.col(0).at<double>(1),punkt.col(0).at<double>(2));
            //wyswietlanie

            x = punkt3d.x;
            y = punkt3d.y;
            z = punkt3d.z;

            psmove_poll(controller);
            unsigned int pressed,released;
            psmove_get_button_events(controller,&pressed,&released);
            psmove_poll(controller);
            unsigned int held = psmove_get_buttons(controller);
            if(psmove_get_trigger(controller) > 0)
            {
                //trigger on
                tool = -1;
                if(released & Btn_SQUARE)//nowa warstwa/shape
                {
                    tool = 8;
                    layer ++;
                    strokesU.clear();
                    strokeTypeU.clear();
                    strokeLayerU.clear();
                    strokeColorU.clear();
                }
                if(held & Btn_CROSS || released & Btn_CROSS)//obracanie
                    tool = 6;
                if(held & Btn_TRIANGLE || released &Btn_TRIANGLE)//przesowanie
                    tool = 7;
                if(held & Btn_CIRCLE || released & Btn_CIRCLE)//kopia
                    tool = 9;
                if(released & Btn_MOVE)//shape mode
                {
                    tool = -1;
                    if(!shapeMode)
                        shapeMode = true;
                    else
                        shapeMode = false;
                }

                //zapisanie pozycji kontrolera
                if(tool != -1 && tool != 8)
                {
                    if(verticies.size() >= 2)
                        verticies.pop_back();
                    verticies.push_back(Point3d(x,y,z));
                }

                //zapisywanie calego ruchu
                if(released & Btn_CROSS
                || released & Btn_TRIANGLE)
                {
                    //jesli shapemode=true to tylko przesowa dany shape
                    if(shapeMode)
                    {
                        strokes.push_back(verticies);
                        strokeType.push_back(tool);
                        strokeLayer.push_back(layer);
                        strokeColor.push_back(color);
                    }else// inaczej przesowa wszystko
                    {
                        for(int i=0;i<layers.size();i++)
                        {
                            strokes.push_back(verticies);
                            strokeType.push_back(tool);
                            strokeLayer.push_back(layers[i]);
                            strokeColor.push_back(color);
                        }
                    }
                    verticies.clear();

                    //new layer used
                    if(find(layers.begin(),layers.end(),layer) == layers.end())
                    {
                        layers.push_back(layer);
                    }
                    //clear undo stack
                    strokesU.clear();
                    strokeTypeU.clear();
                    strokeLayerU.clear();
                    strokeColorU.clear();
                }
                //kopia
                if(released & Btn_CIRCLE && find(layers.begin(),layers.end(),layer) != layers.end())
                {
                    //verticies invert
                    Point3d temporary = verticies.front();
                    verticies.front() = verticies.back();
                    verticies.back() = temporary;
                    //copy marker
                    strokes.push_back(verticies);
                    strokeType.push_back(tool);
                    if(shapeMode == true)
                        strokeLayer.push_back(layer);
                    else
                        strokeLayer.push_back(0);
                    strokeColor.push_back(color);
                    //verticies invert
                    temporary = verticies.front();
                    verticies.front() = verticies.back();
                    verticies.back() = temporary;
                    int outLayer = 0;
                    if(shapeMode)
                        outLayer = layer;

                    int loopEnd = strokes.size();
                    for(int i=0;i<loopEnd-1;i++)
                    {
                        if(strokeLayer[i] != layer)
                            continue;
                        strokes.push_back(strokes[i]);
                        strokeType.push_back(strokeType[i]);
                        strokeLayer.push_back(outLayer);
                        strokeColor.push_back(strokeColor[i]);
                    }
                    //copy marker
                    strokes.push_back(verticies);
                    strokeType.push_back(tool);
                    if(shapeMode == true)
                        strokeLayer.push_back(layer);
                    else
                        strokeLayer.push_back(0);
                    strokeColor.push_back(color);
                    verticies.clear();
                    //clear undo stack
                    strokesU.clear();
                    strokeTypeU.clear();
                    strokeLayerU.clear();
                    strokeColorU.clear();

                }
            }
            else
            {
                //trigger off
                tool = -1;
                if(held & Btn_CROSS || released & Btn_CROSS)//kreska
                    tool = 1;
                else if(held & Btn_SQUARE || released & Btn_SQUARE)//prostopadloscian
                    tool = 0;
                else if(held & Btn_TRIANGLE || released & Btn_SQUARE)//stozek
                    tool = 2;
                else if(held & Btn_CIRCLE || released & Btn_SQUARE)//kula
                    tool = 3;

                //kolor
                if(held & Btn_MOVE)
                {
                    colorMode = true;
                    tool = -1;
                }
                if(released & Btn_MOVE)
                {
                    colorMode = false;
                    tool = -1;
                }
                //zapisanie pozycji kontrolera
                if(tool != -1)
                {
                    if(tool != 1 && verticies.size() >= 2)
                    {
                        verticies.pop_back();
                    }
                    verticies.push_back(Point3d(x,y,z));
                }

                //zapisywanie calego ruchu
                if(released & Btn_SQUARE || released & Btn_CROSS
                || released & Btn_TRIANGLE || released & Btn_CIRCLE)
                {
                    strokes.push_back(verticies);
                    if(released & Btn_SQUARE)
                        tool = 0;
                    if(released & Btn_CROSS)
                        tool = 1;
                    if(released & Btn_TRIANGLE)
                        tool = 2;
                    if(released & Btn_CIRCLE)
                        tool = 3;
                    strokeType.push_back(tool);
                    if(shapeMode == true)
                        strokeLayer.push_back(layer);
                    else
                        strokeLayer.push_back(0);
                    strokeColor.push_back(color);
                    verticies.clear();

                    //new layer used
                    if(find(layers.begin(),layers.end(),layer) == layers.end())
                    {
                        layers.push_back(layer);
                    }

                    strokesU.clear();
                    strokeTypeU.clear();
                    strokeLayerU.clear();
                    strokeColorU.clear();
                }

                if(released & Btn_START && !strokes.empty())
                {
                    //undo
                    if(strokeType.back() != 9)
                    {
                        strokesU.push_back(strokes.back());
                        strokeTypeU.push_back(strokeType.back());
                        strokeLayerU.push_back(strokeLayer.back());
                        strokeColorU.push_back(strokeColor.back());

                        strokes.pop_back();
                        strokeType.pop_back();
                        strokeLayer.pop_back();
                        strokeColor.pop_back();
                    }
                    //undo copy
                    else
                    {
                        //find the begining
                        int copyBegin;
                        Point3d vertex = strokes.back().front();
                        for(copyBegin = strokeType.size()-2;copyBegin>=0;copyBegin--)
                        {
                            if(strokeType[copyBegin] == 9 && vertex == strokes[copyBegin].back())
                                break;
                        }
                        //copy
                        for(int i = copyBegin;i<strokes.size();i++)
                        {
                            strokesU.push_back(strokes[i]);
                            strokeTypeU.push_back(strokeType[i]);
                            strokeLayerU.push_back(strokeLayer[i]);
                            strokeColorU.push_back(strokeColor[i]);
                        }
                        //pop stroke stack
                        int end = strokes.size()-copyBegin;
                        for(int i=0;i<end;i++)
                        {
                            strokes.pop_back();
                            strokeType.pop_back();
                            strokeLayer.pop_back();
                            strokeColor.pop_back();
                        }
                    }
                }
                if(released & Btn_SELECT && !strokesU.empty())
                {
                    //redo
                    if(strokeTypeU.back() != 9)
                    {
                        strokes.push_back(strokesU.back());
                        strokeType.push_back(strokeTypeU.back());
                        strokeLayer.push_back(strokeLayerU.back());
                        strokeColor.push_back(strokeColorU.back());

                        strokesU.pop_back();
                        strokeTypeU.pop_back();
                        strokeLayerU.pop_back();
                        strokeColorU.pop_back();
                    }
                    else
                    {
                        //find the begining
                        int copyBegin;
                        Point3d vertex = strokesU.back().front();
                        for(copyBegin = strokeTypeU.size()-2;copyBegin>=0;copyBegin--)
                        {
                            if(strokeTypeU[copyBegin] == 9 && vertex == strokesU[copyBegin].back())
                                break;
                        }
                        //copy
                        for(int i = copyBegin;i<strokesU.size();i++)
                        {
                            strokes.push_back(strokesU[i]);
                            strokeType.push_back(strokeTypeU[i]);
                            strokeLayer.push_back(strokeLayerU[i]);
                            strokeColor.push_back(strokeColorU[i]);
                        }
                        //pop undo stack
                        int end = strokesU.size()-copyBegin;
                        for(int i=0;i<end;i++)
                        {
                            strokesU.pop_back();
                            strokeTypeU.pop_back();
                            strokeLayerU.pop_back();
                            strokeColorU.pop_back();
                        }
                    }
                }
            }

        }

        //qDebug()<< "(" << x<< "," <<y << "," <<z << ")";
        glEnable( GL_TEXTURE_2D );
        background(texture);
        glDisable(GL_TEXTURE_2D);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float fy = camMat1.at<double>(1,1);
        float fovy = atan(frame1local.rows/(2*fy))*360/CV_PI;
        float aspect = frame1local.rows/frame1local.cols;
        float near = 0.001;
        float far = 100.0;
        gluPerspective(fovy, aspect, near, far);
        //192.168.43.48 249
        //ustawienie modelview matrix
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        Mat dir,up;
        vector<double> vdir = {0,0,1,1};
        vector<double> vup = {0,1,0,1};
        dir.push_back(vdir);
        up.push_back(vup);
        dir = T1 * dir;
        up = R1 * up;
        dir = dir / dir.col(0).at<double>(3);
        up = up / up.col(0).at<double>(3);
        gluLookAt(camPoint1.x,camPoint1.y,camPoint1.z,dir.col(0).at<double>(0),dir.col(0).at<double>(1),dir.col(0).at<double>(2),up.col(0).at<double>(0),up.col(0).at<double>(1),up.col(0).at<double>(2));
        glEnable(GL_DEPTH_TEST);
        if(colorMode)
        {
            //color picking mode
            //lights(-1,0,0,0,0,0,0);//fullbright
            double scale = 0.2;// 0.5 boku palety kolorow
            //wyliczanie rogow plaszczyzny
            vector<double> vtr = {scale,scale,0,1},vtl = {-scale,scale,0,1},vbr = {scale,-scale,0,1},vbl = {-scale,-scale,0,1};
            Mat mtr,mtl,mbr,mbl;
            mtr.push_back(vtr);
            mtl.push_back(vtl);
            mbr.push_back(vbr);
            mbl.push_back(vbl);
            Point3d tr(mtr.col(0).at<double>(0)/mtr.col(0).at<double>(3),mtr.col(0).at<double>(1)/mtr.col(0).at<double>(3),mtr.col(0).at<double>(2)/mtr.col(0).at<double>(3));
            Point3d tl(mtl.col(0).at<double>(0)/mtl.col(0).at<double>(3),mtl.col(0).at<double>(1)/mtl.col(0).at<double>(3),mtl.col(0).at<double>(2)/mtl.col(0).at<double>(3));
            Point3d br(mbr.col(0).at<double>(0)/mbr.col(0).at<double>(3),mbr.col(0).at<double>(1)/mbr.col(0).at<double>(3),mbr.col(0).at<double>(2)/mbr.col(0).at<double>(3));
            Point3d bl(mbl.col(0).at<double>(0)/mbl.col(0).at<double>(3),mbl.col(0).at<double>(1)/mbl.col(0).at<double>(3),mbl.col(0).at<double>(2)/mbl.col(0).at<double>(3));
            //rysowanie plaszczyzny
            //przezroczystosc nie dziala!!!
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable( GL_TEXTURE_2D );
            glDeleteTextures(1,&colorWheel);
            colorWheel = LoadTexture(wheel);
            glBindTexture(GL_TEXTURE_2D,colorWheel);
            glBegin(GL_QUADS);
                glTexCoord2d(0,0);glVertex3d(tl.x,tl.y,tl.z);
                glTexCoord2d(0,1);glVertex3d(bl.x,bl.y,bl.z);
                glTexCoord2d(1,1);glVertex3d(br.x,br.y,br.z);
                glTexCoord2d(1,0);glVertex3d(tr.x,tr.y,tr.z);
            glEnd();
            glDisable(GL_TEXTURE_2D );
            glDisable(GL_BLEND);

            //sprawdzenie koloru
            //obliczanie przeciecia palety i lini biegnacej z kontrolera
            //double s = (up.col(0).at<double>(0)*x+up.col(0).at<double>(1)*y+up.col(0).at<double>(2)*z)/(up.dot(up));
            Point3d P0 = Point3d(x,y,z);
            Point3d upPoint = Point3d(up.col(0).at<double>(0),up.col(0).at<double>(1),up.col(0).at<double>(2));
            double s  = P0.z/upPoint.z;
            Point3d intersection = P0 - s*upPoint;
            //obliczanie vectora prawy gorny rog, przeciecie

            Vec3d imageVec2d = intersection - tl;
            int imageX = (int)(imageVec2d[0] * (wheel.cols/(2*scale)));
            int imageY = -(int)(imageVec2d[1] * (wheel.cols/(2*scale)));
            //qDebug() << imageX <<imageY;
            if(imageX > 0 && imageX < wheel.cols &&
                    imageY > 0 && imageY < wheel.cols)
            {
                Vec4b colorVec = wheel.at<Vec4b>(Point(imageX,imageY));
                color = Point3d((double)colorVec[2]/255,(double)colorVec[1]/255,(double)colorVec[0]/255);
            }
            //rysowanie pozycji kontrollera
            glPushMatrix();
            glTranslated(x,y,z);
            glutSolidSphere(size,6,6);
            glPopMatrix();
            //rysowanie lini pomocniczej
            glEnable(GL_COLOR_MATERIAL);
            glColor3d(color.x,color.y,color.z);
            glBegin(GL_LINES);
            glVertex3d(intersection.x,intersection.y,intersection.z);
            glVertex3d(x,y,z);
            glEnd();
            glColor3f(1,1,1);
            glDisable(GL_COLOR_MATERIAL);
            //end

            contours.clear();
            hierarchy.clear();
            frame1local.release();
            frame2local.release();
            calc.release();
            tex.release();
            glutSwapBuffers();
            glutPostRedisplay();
            return;
        }
        /*if(found&&found2)
    if(image.size() == 0)
        {
            glPointSize(5.0);
            glLineWidth(5.0);
            glMatrixMode(GL_MODELVIEW);
            glEnable(GL_COLOR_MATERIAL);
            glColor3f(1,0,0);
            glBegin(GL_LINES);
            glVertex3d(dir.col(0).at<double>(0),dir.col(0).at<double>(1),dir.col(0).at<double>(2));
            glVertex3d(raay1.x,raay1.y, raay1.z);
            glEnd();
            glMatrixMode(GL_MODELVIEW);
            glColor3f(1,1,1);
            glDisable(GL_COLOR_MATERIAL);
            glEnable(GL_COLOR_MATERIAL);
            glColor3f(0,1,0);
            glBegin(GL_LINES);
            glVertex3d(camPoint2.x, camPoint2.y, camPoint2.z);
            glVertex3d(raay2.x, raay2.y, raay2.z);
            glEnd();
            glColor3f(1,1,1);
            glDisable(GL_COLOR_MATERIAL);
        }*/

        lights(1,(float)dir.col(0).at<double>(0),(float)dir.col(0).at<double>(1),(float)dir.col(0).at<double>(2),(float)camPoint1.x,(float)camPoint1.y,(float)camPoint1.z);

        glEnable(GL_COLOR_MATERIAL);
        glColor3f(color.x,color.y,color.z);
        glPointSize(5);
        glLineWidth(5);
        double angle,rx,ry,rz,tx,ty,tz;
        if(verticies.size()>1)
        {
            if(tool == 1)
            {
                for(int j=0;j<verticies.size()-1;j++)
                {
                    glBegin(GL_LINES);
                    glVertex3d(verticies[j].x,verticies[j].y,verticies[j].z);
                    glVertex3d(verticies[j+1].x,verticies[j+1].y,verticies[j+1].z);
                    glEnd();
                }
            }
            if(tool == 0)
            {
                double a,b,c;
                Vec3d d = verticies.back() - verticies.front();
                a = d[0];
                b = d[1];
                c = d[2];
                Point3d p1 = verticies.front();
                Point3d q1 = p1,q2 = p1,q3 = p1,q4 = p1,q5 = p1,q6 = p1,q7 = p1,q8 = p1;
                q2.x += a;
                q3.x += a;
                q3.y += b;
                q4.y += b;

                q6.y += b;
                q7.x += a;
                q7.y += b;
                q8.x += a;
                q5.z += c;
                q6.z += c;
                q7.z += c;
                q8.z += c;
                glBegin(GL_QUADS);
                    glVertex3d(q1.x,q1.y,q1.z);
                    glVertex3d(q2.x,q2.y,q2.z);
                    glVertex3d(q3.x,q3.y,q3.z);
                    glVertex3d(q4.x,q4.y,q4.z);
                glEnd();
                glBegin(GL_QUADS);
                    glVertex3d(q8.x,q8.y,q8.z);
                    glVertex3d(q7.x,q7.y,q7.z);
                    glVertex3d(q6.x,q6.y,q6.z);
                    glVertex3d(q5.x,q5.y,q5.z);
                glEnd();
                glBegin(GL_QUADS);
                    glVertex3d(q2.x,q2.y,q2.z);//a
                    glVertex3d(q3.x,q3.y,q3.z);//ab
                    glVertex3d(q7.x,q7.y,q7.z);//abc
                    glVertex3d(q8.x,q8.y,q8.z);//ac
                glEnd();
                glBegin(GL_QUADS);
                    glVertex3d(q4.x,q4.y,q4.z);//b
                    glVertex3d(q3.x,q3.y,q3.z);//ab
                    glVertex3d(q7.x,q7.y,q7.z);//abc
                    glVertex3d(q6.x,q6.y,q6.z);//bc
                glEnd();
                glBegin(GL_QUADS);
                    glVertex3d(q1.x,q1.y,q1.z);//
                    glVertex3d(q5.x,q5.y,q5.z);//c
                    glVertex3d(q8.x,q8.y,q8.z);//ac
                    glVertex3d(q2.x,q2.y,q2.z);//a
                glEnd();
                glBegin(GL_QUADS);
                    glVertex3d(q1.x,q1.y,q1.z);//
                    glVertex3d(q5.x,q5.y,q5.z);//c
                    glVertex3d(q6.x,q6.y,q6.z);//bc
                    glVertex3d(q4.x,q4.y,q4.z);//b
                glEnd();
            }
            if(tool == 3)
            {
                glPushMatrix();
                glTranslated(verticies.front().x,verticies.front().y,verticies.front().z);
                Vec3d r = verticies.back() - verticies.front();
                double rlen =  sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
                glutSolidSphere(rlen,12,12);
                glPopMatrix();
            }
            if(tool == 2)
            {
                Vec3d r = verticies.back() - verticies.front();
                double rlen =  sqrt(r[0]*r[0] + r[1]*r[1]);
                GLUquadricObj *quad = gluNewQuadric();

                renderCylinder(verticies.front().x,verticies.front().y,verticies.front().z,
                               verticies.front().x,verticies.front().y,verticies.back().z,rlen,16,quad);
            }
            //calculate rotation
            if(tool == 6 && find(layers.begin(),layers.end(),layer) != layers.end())
            {
                Vec3d v1(verticies.front().x,verticies.front().y,verticies.front().z);
                Vec3d v2(verticies.back().x,verticies.back().y,verticies.back().z);
                double v1size = sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
                double v2size = sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
                angle = acos(v1.ddot(v2)/(v1size*v2size));
                Vec3d v3 = v1.cross(v2);
                rx = v3[0];
                ry = v3[1];
                rz = v3[2];

            }
            //calculate translation
            if((tool == 7 || tool == 9) && find(layers.begin(),layers.end(),layer) != layers.end())
            {
                tx = verticies.back().x - verticies.front().x;
                ty = verticies.back().y - verticies.front().y;
                tz = verticies.back().z - verticies.front().z;
            }
        }
        glPushMatrix();

        if(tool == 6 && !shapeMode)
            glRotated(angle*180/CV_PI,rx,ry,rz);
        if(tool == 7 && !shapeMode)
            glTranslated(tx,ty,tz);

        psmove_poll(controller);
        for(int i=0;i<layers.size();i++)
        {
            glPushMatrix();
            //rotation + translation in while action button is held
            if(tool == 6 && layers[i] == layer && shapeMode)
                glRotated(angle*180/CV_PI,rx,ry,rz);
            if(tool == 7 && layers[i] == layer && shapeMode)
                glTranslated(tx,ty,tz);
            for(int j=strokes.size()-1;j>=0;j--)
            {
                //color
                //xxxzzzyyyyzx
                glColor3f(strokeColor[j].x,strokeColor[j].y,strokeColor[j].z);
                //draw lines
                if(strokeType[j] == 1 && layers[i]==strokeLayer[j])
                {
                    for(int k=0;k<strokes[j].size()-1;k++)
                    {
                        glBegin(GL_LINES);
                        glVertex3d(strokes[j][k].x,strokes[j][k].y,strokes[j][k].z);
                        glVertex3d(strokes[j][k+1].x,strokes[j][k+1].y,strokes[j][k+1].z);
                        glEnd();
                    }
                    double vx = strokes[j].back().x - strokes[j].front().x;
                    double vy = strokes[j].back().y - strokes[j].front().y;
                    double vz = strokes[j].back().z - strokes[j].front().z;
                    double len = sqrt(vx*vx + vy*vy + vz*vz);
                    qDebug() << len;
                }
                //quboid
                if(strokeType[j] == 0 && layers[i] == strokeLayer[j])
                {
                    double a,b,c;
                    Vec3d d = strokes[j].back() - strokes[j].front();
                    a = d[0];
                    b = d[1];
                    c = d[2];
                    Point3d p1 = strokes[j].front();
                    Point3d q1 = p1,q2 = p1,q3 = p1,q4 = p1,q5 = p1,q6 = p1,q7 = p1,q8 = p1;
                    q2.x += a;
                    q3.x += a;
                    q3.y += b;
                    q4.y += b;

                    q6.y += b;
                    q7.x += a;
                    q7.y += b;
                    q8.x += a;
                    q5.z += c;
                    q6.z += c;
                    q7.z += c;
                    q8.z += c;
                    glBegin(GL_QUADS);
                        glVertex3d(q1.x,q1.y,q1.z);
                        glVertex3d(q2.x,q2.y,q2.z);
                        glVertex3d(q3.x,q3.y,q3.z);
                        glVertex3d(q4.x,q4.y,q4.z);
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q8.x,q8.y,q8.z);
                        glVertex3d(q7.x,q7.y,q7.z);
                        glVertex3d(q6.x,q6.y,q6.z);
                        glVertex3d(q5.x,q5.y,q5.z);
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q2.x,q2.y,q2.z);//a
                        glVertex3d(q3.x,q3.y,q3.z);//ab
                        glVertex3d(q7.x,q7.y,q7.z);//abc
                        glVertex3d(q8.x,q8.y,q8.z);//ac
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q4.x,q4.y,q4.z);//b
                        glVertex3d(q3.x,q3.y,q3.z);//ab
                        glVertex3d(q7.x,q7.y,q7.z);//abc
                        glVertex3d(q6.x,q6.y,q6.z);//bc
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q1.x,q1.y,q1.z);//
                        glVertex3d(q5.x,q5.y,q5.z);//c
                        glVertex3d(q8.x,q8.y,q8.z);//ac
                        glVertex3d(q2.x,q2.y,q2.z);//a
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q1.x,q1.y,q1.z);//
                        glVertex3d(q5.x,q5.y,q5.z);//c
                        glVertex3d(q6.x,q6.y,q6.z);//bc
                        glVertex3d(q4.x,q4.y,q4.z);//b
                    glEnd();
                }
                //sphere
                if(strokeType[j] == 3 && layers[i] == strokeLayer[j])
                {
                    glPushMatrix();
                    glTranslated(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z);
                    Vec3d r = strokes[j].back() - strokes[j].front();
                    double rlen =  sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
                    glutSolidSphere(rlen,12,12);
                    glPopMatrix();
                }
                //cone
                if(strokeType[j] == 2 && layers[i] == strokeLayer[j])
                {
                    Vec3d r = strokes[j].back() - strokes[j].front();
                    double rlen =  sqrt(r[0]*r[0] + r[1]*r[1]);
                    GLUquadricObj *quad = gluNewQuadric();

                    renderCylinder(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z,
                                   strokes[j].front().x,strokes[j].front().y,strokes[j].back().z,rlen,16,quad);
                }
                //rotate
                if(strokeType[j] == 6&&layers[i]==strokeLayer[j])
                {
                    Vec3d v1(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z);
                    Vec3d v2(strokes[j].back().x,strokes[j].back().y,strokes[j].back().z);
                    double v1size = sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
                    double v2size = sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
                    double angleStroke = acos(v1.ddot(v2)/(v1size*v2size));
                    Vec3d v3 = v1.cross(v2);
                    double rxStroke = v3[0];
                    double ryStroke = v3[1];
                    double rzStroke = v3[2];
                    glRotated(angleStroke*180/CV_PI,rxStroke,ryStroke,rzStroke);
                }
                //translate
                if((strokeType[j] == 7 || strokeType[j] == 9)&&layers[i]==strokeLayer[j])
                {
                    glTranslated(strokes[j].back().x - strokes[j].front().x,
                                 strokes[j].back().y - strokes[j].front().y,
                                 strokes[j].back().z - strokes[j].front().z);
                }
            }
            glPopMatrix();
            if(layers[i] == layer && tool == 9)
            {
                glPushMatrix();
                glTranslated(tx,ty,tz);
                for(int j=strokes.size()-1;j>=0;j--)
                {
                    //color
                    glColor3f(strokeColor[j].x,strokeColor[j].y,strokeColor[j].z);
                    //draw lines
                    if(strokeType[j] == 1 && layers[i]==strokeLayer[j])
                    {
                        for(int k=0;k<strokes[j].size()-1;k++)
                        {
                            glBegin(GL_LINES);
                            glVertex3d(strokes[j][k].x,strokes[j][k].y,strokes[j][k].z);
                            glVertex3d(strokes[j][k+1].x,strokes[j][k+1].y,strokes[j][k+1].z);
                            glEnd();
                        }
                    }
                    //quboid
                    if(strokeType[j] == 0 && layers[i] == strokeLayer[j])
                    {
                        double a,b,c;
                        Vec3d d = strokes[j].back() - strokes[j].front();
                        a = d[0];
                        b = d[1];
                        c = d[2];
                        Point3d p1 = strokes[j].front();
                        Point3d q1 = p1,q2 = p1,q3 = p1,q4 = p1,q5 = p1,q6 = p1,q7 = p1,q8 = p1;
                        q2.x += a;
                        q3.x += a;
                        q3.y += b;
                        q4.y += b;

                        q6.y += b;
                        q7.x += a;
                        q7.y += b;
                        q8.x += a;
                        q5.z += c;
                        q6.z += c;
                        q7.z += c;
                        q8.z += c;
                        glBegin(GL_QUADS);
                            glVertex3d(q1.x,q1.y,q1.z);
                            glVertex3d(q2.x,q2.y,q2.z);
                            glVertex3d(q3.x,q3.y,q3.z);
                            glVertex3d(q4.x,q4.y,q4.z);
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q8.x,q8.y,q8.z);
                            glVertex3d(q7.x,q7.y,q7.z);
                            glVertex3d(q6.x,q6.y,q6.z);
                            glVertex3d(q5.x,q5.y,q5.z);
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q2.x,q2.y,q2.z);//a
                            glVertex3d(q3.x,q3.y,q3.z);//ab
                            glVertex3d(q7.x,q7.y,q7.z);//abc
                            glVertex3d(q8.x,q8.y,q8.z);//ac
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q4.x,q4.y,q4.z);//b
                            glVertex3d(q3.x,q3.y,q3.z);//ab
                            glVertex3d(q7.x,q7.y,q7.z);//abc
                            glVertex3d(q6.x,q6.y,q6.z);//bc
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q1.x,q1.y,q1.z);//
                            glVertex3d(q5.x,q5.y,q5.z);//c
                            glVertex3d(q8.x,q8.y,q8.z);//ac
                            glVertex3d(q2.x,q2.y,q2.z);//a
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q1.x,q1.y,q1.z);//
                            glVertex3d(q5.x,q5.y,q5.z);//c
                            glVertex3d(q6.x,q6.y,q6.z);//bc
                            glVertex3d(q4.x,q4.y,q4.z);//b
                        glEnd();
                    }
                    //sphere
                    if(strokeType[j] == 3 && layers[i] == strokeLayer[j])
                    {
                        glPushMatrix();
                        glTranslated(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z);
                        Vec3d r = strokes[j].back() - strokes[j].front();
                        double rlen =  sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
                        glutSolidSphere(rlen,12,12);
                        glPopMatrix();
                    }
                    //cone
                    if(strokeType[j] == 2 && layers[i] == strokeLayer[j])
                    {
                        Vec3d r = strokes[j].back() - strokes[j].front();
                        double rlen =  sqrt(r[0]*r[0] + r[1]*r[1]);
                        GLUquadricObj *quad = gluNewQuadric();

                        renderCylinder(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z,
                                       strokes[j].front().x,strokes[j].front().y,strokes[j].back().z,rlen,16,quad);
                    }
                    //rotate
                    if(strokeType[j] == 6&&layers[i]==strokeLayer[j])
                    {
                        Vec3d v1(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z);
                        Vec3d v2(strokes[j].back().x,strokes[j].back().y,strokes[j].back().z);
                        double v1size = sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
                        double v2size = sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
                        double angleStroke = acos(v1.ddot(v2)/(v1size*v2size));
                        Vec3d v3 = v1.cross(v2);
                        double rxStroke = v3[0];
                        double ryStroke = v3[1];
                        double rzStroke = v3[2];
                        glRotated(angleStroke*180/CV_PI,rxStroke,ryStroke,rzStroke);
                    }
                    //translate
                    if((strokeType[j] == 7 || strokeType[j] == 9)&&layers[i]==strokeLayer[j])
                    {
                        glTranslated(strokes[j].back().x - strokes[j].front().x,
                                     strokes[j].back().y - strokes[j].front().y,
                                     strokes[j].back().z - strokes[j].front().z);
                    }
                }
                glPopMatrix();
            }
        }
        glPopMatrix();
        glDisable(GL_COLOR_MATERIAL);

        glPushMatrix();
        glTranslated(x,y,z);
        glEnable(GL_COLOR_MATERIAL);
        psmove_poll(controller);
        if(psmove_get_buttons(controller) & Btn_MOVE)
            glColor3f(1,0,0);
        else
            glColor3f(1,1,1);
        glutSolidSphere(size,6,6);
        glColor3f(1,1,1);
        glDisable(GL_COLOR_MATERIAL);
        glPopMatrix();

    }
    else
    {
        glEnable( GL_TEXTURE_2D );
        background(texture);
        glDisable(GL_TEXTURE_2D);
    }
    lights(-1,0,0,0,0,0,0);
    contours.clear();
    hierarchy.clear();
    frame1local.release();
    frame2local.release();
    calc.release();
    tex.release();
    glutSwapBuffers();
    glutPostRedisplay();
}

void GLRender::draw3()
{
    usleep(1000);
    glutSwapBuffers();
    glutPostRedisplay();
}

//unfinished
void GLRender::draw2()
{
    if(!win3valid){
        usleep(10000);
        return;
    }
    isArucoCalibrated3 =false;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    Mat frame3local;

        tvec3.clear();
        rvec3.clear();
        Mat local3;
        mut3.lock();
        frame3.copyTo(local3);
        mut3.unlock();

        if(local3.empty())
        {
            glutPostRedisplay();
            return;
        }

        flip(local3,local3,1);
        vector<int> markerIDs;
        markerIDs.push_back(0);
        Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
        vector<vector<Point2f>> markercorners1,rej1;
        aruco::detectMarkers(local3,markerDictionary,markercorners1,markerIDs,parametersPtr,rej1);//,camMat1,coeff1);
        aruco::estimatePoseSingleMarkers(markercorners1,arucoDims,camMat3,coeff3,rvec3,tvec3);

        if(rvec3.size()>0)
            isArucoCalibrated3 = true;

    mut3.lock();
    frame3local = frame3;
    mut3.unlock();

    glDeleteTextures(1,&texture2);
    texture2  = LoadTexture(frame3local);

    Mat rotation,translation;

    if(isArucoCalibrated3)
    {
        Mat rmat3;
        Rodrigues(rvec3[0],rmat3);
        Mat t3;
        t3.push_back(tvec3[0][0]);
        t3.push_back(tvec3[0][1]);
        t3.push_back(tvec3[0][2]);
        Mat z3(1,3,t3.type());
        z3.col(0).at<double>(0) = 0;
        z3.col(0).at<double>(1) = 0;
        z3.col(0).at<double>(2) = 0;
        z3 = z3.t();
        vector<double> l4 = {0,0,0,1};
        Mat T3,R3;
        hconcat(rmat3,t3,T3);
        hconcat(rmat3,z3,R3);
        R3.push_back(Mat(l4).t());
        T3.push_back(Mat(l4).t());
        T3 = T3.inv();
        R3 = R3.inv();
        glEnable( GL_TEXTURE_2D );
        background(texture2);
        glDisable(GL_TEXTURE_2D);


        Mat cam3coords;
        Mat zero; zero.push_back(l4);
        cam3coords = T3 * zero;
        Point3d camPoint3(cam3coords.col(0).at<double>(0)/cam3coords.col(0).at<double>(3),cam3coords.col(0).at<double>(1)/cam3coords.col(0).at<double>(3),cam3coords.col(0).at<double>(2)/cam3coords.col(0).at<double>(3));

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float fy = camMat3.at<double>(1,1);
        float fovy = atan(320/(2*fy))*360/CV_PI;
        float aspect = 240.0/320.0;
        float near = 0.00001;
        float far = 100.0;
        gluPerspective(fovy, aspect, near, far); //fov = 70.05 daje dobre efekty
        //192.168.43.48 249
        //ustawienie modelview matrix
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        Mat dir,up;
        vector<double> vdir = {0,0,1,1};
        vector<double> vup = {0,1,0,1};
        dir.push_back(vdir);
        up.push_back(vup);
        dir = T3 * dir;
        up = R3 * up;
        dir = dir / dir.col(0).at<double>(3);
        up = up / up.col(0).at<double>(3);
        gluLookAt(camPoint3.x,camPoint3.y,camPoint3.z,dir.col(0).at<double>(0),dir.col(0).at<double>(1),dir.col(0).at<double>(2),up.col(0).at<double>(0),up.col(0).at<double>(1),up.col(0).at<double>(2));

        lights(1,(float)dir.col(0).at<double>(0),(float)dir.col(0).at<double>(1),(float)dir.col(0).at<double>(2),(float)camPoint3.x,(float)camPoint3.y,(float)camPoint3.z);

        glEnable(GL_COLOR_MATERIAL);
        glColor3f(color.x,color.y,color.z);
        glPointSize(5);
        glLineWidth(5);
        double angle,rx,ry,rz,tx,ty,tz;

        glPushMatrix();

        psmove_poll(controller);
        for(int i=0;i<layers.size();i++)
        {
            glPushMatrix();

            for(int j=strokes.size()-1;j>=0;j--)
            {
                //color
                glColor3f(strokeColor[j].x,strokeColor[j].y,strokeColor[j].z);
                //draw lines
                if(strokeType[j] == 1 && layers[i]==strokeLayer[j])
                {
                    for(int k=0;k<strokes[j].size()-1;k++)
                    {
                        glBegin(GL_LINES);
                        glVertex3d(strokes[j][k].x,strokes[j][k].y,strokes[j][k].z);
                        glVertex3d(strokes[j][k+1].x,strokes[j][k+1].y,strokes[j][k+1].z);
                        glEnd();
                    }
                }
                //quboid
                if(strokeType[j] == 0 && layers[i] == strokeLayer[j])
                {
                    double a,b,c;
                    Vec3d d = strokes[j].back() - strokes[j].front();
                    a = d[0];
                    b = d[1];
                    c = d[2];
                    Point3d p1 = strokes[j].front();
                    Point3d q1 = p1,q2 = p1,q3 = p1,q4 = p1,q5 = p1,q6 = p1,q7 = p1,q8 = p1;
                    q2.x += a;
                    q3.x += a;
                    q3.y += b;
                    q4.y += b;

                    q6.y += b;
                    q7.x += a;
                    q7.y += b;
                    q8.x += a;
                    q5.z += c;
                    q6.z += c;
                    q7.z += c;
                    q8.z += c;
                    glBegin(GL_QUADS);
                        glVertex3d(q1.x,q1.y,q1.z);
                        glVertex3d(q2.x,q2.y,q2.z);
                        glVertex3d(q3.x,q3.y,q3.z);
                        glVertex3d(q4.x,q4.y,q4.z);
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q8.x,q8.y,q8.z);
                        glVertex3d(q7.x,q7.y,q7.z);
                        glVertex3d(q6.x,q6.y,q6.z);
                        glVertex3d(q5.x,q5.y,q5.z);
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q2.x,q2.y,q2.z);//a
                        glVertex3d(q3.x,q3.y,q3.z);//ab
                        glVertex3d(q7.x,q7.y,q7.z);//abc
                        glVertex3d(q8.x,q8.y,q8.z);//ac
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q4.x,q4.y,q4.z);//b
                        glVertex3d(q3.x,q3.y,q3.z);//ab
                        glVertex3d(q7.x,q7.y,q7.z);//abc
                        glVertex3d(q6.x,q6.y,q6.z);//bc
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q1.x,q1.y,q1.z);//
                        glVertex3d(q5.x,q5.y,q5.z);//c
                        glVertex3d(q8.x,q8.y,q8.z);//ac
                        glVertex3d(q2.x,q2.y,q2.z);//a
                    glEnd();
                    glBegin(GL_QUADS);
                        glVertex3d(q1.x,q1.y,q1.z);//
                        glVertex3d(q5.x,q5.y,q5.z);//c
                        glVertex3d(q6.x,q6.y,q6.z);//bc
                        glVertex3d(q4.x,q4.y,q4.z);//b
                    glEnd();
                }
                //sphere
                if(strokeType[j] == 3 && layers[i] == strokeLayer[j])
                {
                    glPushMatrix();
                    glTranslated(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z);
                    Vec3d r = strokes[j].back() - strokes[j].front();
                    double rlen =  sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
                    glutSolidSphere(rlen,12,12);
                    glPopMatrix();
                }
                //cone
                if(strokeType[j] == 2 && layers[i] == strokeLayer[j])
                {
                    Vec3d r = strokes[j].back() - strokes[j].front();
                    double rlen =  sqrt(r[0]*r[0] + r[1]*r[1]);
                    GLUquadricObj *quad = gluNewQuadric();

                    renderCylinder(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z,
                                   strokes[j].front().x,strokes[j].front().y,strokes[j].back().z,rlen,16,quad);
                }
                //rotate
                if(strokeType[j] == 6&&layers[i]==strokeLayer[j])
                {
                    Vec3d v1(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z);
                    Vec3d v2(strokes[j].back().x,strokes[j].back().y,strokes[j].back().z);
                    double v1size = sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
                    double v2size = sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
                    double angleStroke = acos(v1.ddot(v2)/(v1size*v2size));
                    Vec3d v3 = v1.cross(v2);
                    double rxStroke = v3[0];
                    double ryStroke = v3[1];
                    double rzStroke = v3[2];
                    glRotated(angleStroke*180/CV_PI,rxStroke,ryStroke,rzStroke);
                }
                //translate
                if((strokeType[j] == 7 || strokeType[j] == 9)&&layers[i]==strokeLayer[j])
                {
                    glTranslated(strokes[j].back().x - strokes[j].front().x,
                                 strokes[j].back().y - strokes[j].front().y,
                                 strokes[j].back().z - strokes[j].front().z);
                }
            }
            glPopMatrix();
            if(layers[i] == layer && tool == 9)
            {
                glPushMatrix();
                glTranslated(tx,ty,tz);
                for(int j=strokes.size()-1;j>=0;j--)
                {
                    //color
                    glColor3f(strokeColor[j].x,strokeColor[j].y,strokeColor[j].z);
                    //draw lines
                    if(strokeType[j] == 1 && layers[i]==strokeLayer[j])
                    {
                        for(int k=0;k<strokes[j].size()-1;k++)
                        {
                            glBegin(GL_LINES);
                            glVertex3d(strokes[j][k].x,strokes[j][k].y,strokes[j][k].z);
                            glVertex3d(strokes[j][k+1].x,strokes[j][k+1].y,strokes[j][k+1].z);
                            glEnd();
                        }
                    }
                    //quboid
                    if(strokeType[j] == 0 && layers[i] == strokeLayer[j])
                    {
                        double a,b,c;
                        Vec3d d = strokes[j].back() - strokes[j].front();
                        a = d[0];
                        b = d[1];
                        c = d[2];
                        Point3d p1 = strokes[j].front();
                        Point3d q1 = p1,q2 = p1,q3 = p1,q4 = p1,q5 = p1,q6 = p1,q7 = p1,q8 = p1;
                        q2.x += a;
                        q3.x += a;
                        q3.y += b;
                        q4.y += b;

                        q6.y += b;
                        q7.x += a;
                        q7.y += b;
                        q8.x += a;
                        q5.z += c;
                        q6.z += c;
                        q7.z += c;
                        q8.z += c;
                        glBegin(GL_QUADS);
                            glVertex3d(q1.x,q1.y,q1.z);
                            glVertex3d(q2.x,q2.y,q2.z);
                            glVertex3d(q3.x,q3.y,q3.z);
                            glVertex3d(q4.x,q4.y,q4.z);
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q8.x,q8.y,q8.z);
                            glVertex3d(q7.x,q7.y,q7.z);
                            glVertex3d(q6.x,q6.y,q6.z);
                            glVertex3d(q5.x,q5.y,q5.z);
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q2.x,q2.y,q2.z);//a
                            glVertex3d(q3.x,q3.y,q3.z);//ab
                            glVertex3d(q7.x,q7.y,q7.z);//abc
                            glVertex3d(q8.x,q8.y,q8.z);//ac
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q4.x,q4.y,q4.z);//b
                            glVertex3d(q3.x,q3.y,q3.z);//ab
                            glVertex3d(q7.x,q7.y,q7.z);//abc
                            glVertex3d(q6.x,q6.y,q6.z);//bc
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q1.x,q1.y,q1.z);//
                            glVertex3d(q5.x,q5.y,q5.z);//c
                            glVertex3d(q8.x,q8.y,q8.z);//ac
                            glVertex3d(q2.x,q2.y,q2.z);//a
                        glEnd();
                        glBegin(GL_QUADS);
                            glVertex3d(q1.x,q1.y,q1.z);//
                            glVertex3d(q5.x,q5.y,q5.z);//c
                            glVertex3d(q6.x,q6.y,q6.z);//bc
                            glVertex3d(q4.x,q4.y,q4.z);//b
                        glEnd();
                    }
                    //sphere
                    if(strokeType[j] == 3 && layers[i] == strokeLayer[j])
                    {
                        glPushMatrix();
                        glTranslated(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z);
                        Vec3d r = strokes[j].back() - strokes[j].front();
                        double rlen =  sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
                        glutSolidSphere(rlen,12,12);
                        glPopMatrix();
                    }
                    //cone
                    if(strokeType[j] == 2 && layers[i] == strokeLayer[j])
                    {
                        Vec3d r = strokes[j].back() - strokes[j].front();
                        double rlen =  sqrt(r[0]*r[0] + r[1]*r[1]);
                        GLUquadricObj *quad = gluNewQuadric();

                        renderCylinder(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z,
                                       strokes[j].front().x,strokes[j].front().y,strokes[j].back().z,rlen,16,quad);
                    }
                    //rotate
                    if(strokeType[j] == 6&&layers[i]==strokeLayer[j])
                    {
                        Vec3d v1(strokes[j].front().x,strokes[j].front().y,strokes[j].front().z);
                        Vec3d v2(strokes[j].back().x,strokes[j].back().y,strokes[j].back().z);
                        double v1size = sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
                        double v2size = sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
                        double angleStroke = acos(v1.ddot(v2)/(v1size*v2size));
                        Vec3d v3 = v1.cross(v2);
                        double rxStroke = v3[0];
                        double ryStroke = v3[1];
                        double rzStroke = v3[2];
                        glRotated(angleStroke*180/CV_PI,rxStroke,ryStroke,rzStroke);
                    }
                    //translate
                    if((strokeType[j] == 7 || strokeType[j] == 9)&&layers[i]==strokeLayer[j])
                    {
                        glTranslated(strokes[j].back().x - strokes[j].front().x,
                                     strokes[j].back().y - strokes[j].front().y,
                                     strokes[j].back().z - strokes[j].front().z);
                    }
                }
                glPopMatrix();
            }
        }
        glPopMatrix();
        glDisable(GL_COLOR_MATERIAL);

        glColor3d(1,1,1);

    }
    else
    {
        glEnable( GL_TEXTURE_2D );
        background(texture2);
        glDisable(GL_TEXTURE_2D);
    }
    lights(-1,0,0,0,0,0,0);
    glPopMatrix();
    glutSwapBuffers();
    glutPostRedisplay();
}
//192.168.43.48
//192.168.43.249
//10.149.24.199
void GLRender::reshape(GLsizei width, GLsizei height)
{
    //zmienic zeby wczytywalo odpowiednie macieze!!!

    // GLsizei for non-negative integer
    // Compute aspect ratio of the new window
    if (height == 0) height = 1;                // To prevent divide by 0

    h1 = height;
    w1 = 240.0 * ((float)h1/320.0);
}

void GLRender::vidCap1()
{
    Mat frame;
    while(1)
    {
        cap1.grab();
        cap1.retrieve(frame,0);
        if(mut1.try_lock() == false)
        {
            frame.release();
            usleep(10000);
            continue;
        }

        rotate(frame,frame,0);
        flip(frame,frame,0);
        frame1.release();
        frame1 = frame;
        mut1.unlock();
        frame.release();
    }
}

void extern GLRender::vidCap2()
{
    Mat frame;
    while(1)
    {
        cap2.grab();
        cap2.retrieve(frame);
        if(mut2.try_lock() == false){
            frame.release();
            usleep(10000);
            continue;
        }

        rotate(frame,frame,0);
        flip(frame,frame,0);
        frame2.release();
        frame2 = frame;
        mut2.unlock();
        frame.release();
    }
}

void extern GLRender::vidCap3()
{
    Mat frame;
    while(1)
    {
        cap3.grab();
        cap3.retrieve(frame);
        if(mut3.try_lock() == false){
            frame.release();
            usleep(10000);
            continue;
        }

        rotate(frame,frame,0);
        flip(frame,frame,0);
        frame3.release();
        frame3 = frame;
        mut3.unlock();
        frame.release();
    }
}

void GLRender::win1o()
{
    glutInitWindowSize(450, 600);
    glutInitWindowPosition(100, 100);
    window = glutCreateWindow("win1");
    glutDisplayFunc(draw);
    glutReshapeFunc(reshape);
    win1valid = true;
}

void GLRender::win3o()
{
    glutInitWindowSize(450, 600);
    glutInitWindowPosition(100, 100);
    win3 = glutCreateWindow("win3");
    glutDisplayFunc(draw2);
    glutReshapeFunc(reshape);
    win3valid = true;
}

void GLRender::win1c()
{
    win1valid = false;
    usleep(100000);
    glutDestroyWindow(window);
}

void GLRender::win3c()
{
    win3valid = false;
    usleep(100000);
    glutDestroyWindow(win3);
}

void GLRender::win1d(int arg)
{
    if(arg == 2)
        win1debugmode = true;
    else
        win1debugmode = false;
}

int GLRender::ipChanged()
{
    if(win1valid)
        win1c();

    if(win3valid)
        win3c();
    usleep(100000);
    for ( int i = 0; i < threads.size(); i++ )
    {
            delete threads[i];
    }
    threads.clear();

    if(vidCapAsynch1() == 0 && vidCapAsynch2() == 0 && vidCapAsynch3() == 0)
        return  0;
    else
    {
        kill();
        return 1;
    }
}

void GLRender::kill()
{
    if(win1valid)
        win1c();

    if(win3valid)
        win3c();

    for(std::thread *t : threads)
    {
        delete t;
    }
}

int GLRender::vidCapAsynch1()
{
    cap1 = VideoCapture("http://" + ip1.toStdString() + ":4747/mjpeg");
    if(!cap1.isOpened())
    {
        qDebug() << "IPCAM_1: couldn't establish connection";
        return 1;
    }
    threads.push_back(new thread(vidCap1));
    threads.back()->detach();

    return 0;
}

int GLRender::vidCapAsynch2()
{
    cap2 = VideoCapture("http://" + ip2.toStdString() + ":4747/mjpeg");
    if(!cap2.isOpened())
    {
        qDebug() << "IPCAM_2: couldn't establish connection";
        return 1;
    }

    threads.push_back(new thread(vidCap2));
    threads.back()->detach();

    return 0;
}

int GLRender::vidCapAsynch3()
{
    cap3 = VideoCapture("http://" + ip3.toStdString() + ":4747/mjpeg");
    if(!cap3.isOpened())
    {
        qDebug() << "IPCAM_3: couldn't establish connection";
        return 1;
    }
    threads.push_back(new thread(vidCap3));
    threads.back()->detach();

    return 0;
}

void GLRender::loadCalib()
{
    FileStorage calibration("calibration.yml",FileStorage::READ);
    calibration["camMat1"] >> camMat1;
    calibration["camMat2"] >> camMat2;
    calibration["camMat3"] >> camMat3;
    calibration["coeff1"] >> coeff1;
    calibration["coeff2"] >> coeff2;
    calibration["coeff3"] >> coeff3;

    calibration.release();
}

void GLRender::createKnownBoardPositions(Size boardSize, float edgeLengt, std::vector<Point3f> &corners)
{
    for(int i = 0;i < boardSize.height; i++)
    {
        for(int j = 0;j < boardSize.width; j ++)
        {
            corners.push_back(Point3f(j * edgeLengt, i * edgeLengt, 0.0f));
        }
    }
}

void GLRender::getChessboardCorners(std::vector<Mat> images, std::vector<std::vector<Point2f> > &foundCorners, bool showResults)
{
    for(std::vector<Mat>::iterator i = images.begin(); i != images.end(); i ++)
    {
        std::vector<cv::Point2f> pointBuffer;
        bool found = findChessboardCorners(*i , chessboardDimensions, pointBuffer, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if(found)
        {
            foundCorners.push_back(pointBuffer);
        }
        if(showResults)
        {
            drawChessboardCorners(*i,chessboardDimensions,pointBuffer,found);
            imshow("Looking 4 corners",*i);
            waitKey(0);
        }
    }
}

int GLRender::calibrate1()
{
    Size imageSize;
    Mat rotationMatrix, translationVector, essentialMatrix;

    vector<vector<Point2f>> markedCorners1, rejectedCandidates1;

    cap1 = VideoCapture("http://" + ip1.toStdString() + ":4747/mjpeg");
    if(!cap1.isOpened())
    {
        qDebug() << "IPCAM_1: couldn't establish connection";
        return 1;
    }

    Mat tempPic;
    cap1.read(tempPic);
    imageSize = tempPic.size();

    vector<vector<Point2f>> checherboardImageSpacePoints1;
    getChessboardCorners(calibPics1, checherboardImageSpacePoints1,false);
    vector<vector<Point3f>> worldSpaceCornerPoints1;
    coeff1 = Mat::zeros(8,1, 64);
    Mat Error;
    worldSpaceCornerPoints1.resize(calibPics1.size());
    for(int i = 0; i < calibPics1.size(); i++ )
    {
            for(int j = 0; j < chessboardDimensions.height; j++ ){
                for(int k = 0; k < chessboardDimensions.width; k++ ){
                    worldSpaceCornerPoints1[i].push_back(Point3f(k*calibrationSquareDimention, -j*calibrationSquareDimention, 0));
                }
            }
    }
    camMat1 = initCameraMatrix2D(worldSpaceCornerPoints1,checherboardImageSpacePoints1,imageSize,0);
    double err1 = calibrateCamera(worldSpaceCornerPoints1, checherboardImageSpacePoints1, chessboardDimensions, camMat1, coeff1, rvecs1, tvecs1);

    qDebug() << "calibration finished!";
    qDebug() << "rms:" << err1;
    return 0;
}

int GLRender::calibrate2()
{
    Size imageSize;
    Mat rotationMatrix, translationVector, essentialMatrix;

    vector<vector<Point2f>> markedCorners1, rejectedCandidates1;

    cap2 = VideoCapture("http://" + ip2.toStdString() + ":4747/mjpeg");
    if(!cap2.isOpened())
    {
        qDebug() << "IPCAM_2: couldn't establish connection";
        return 1;
    }

    Mat tempPic;
    cap1.read(tempPic);
    imageSize = tempPic.size();

    vector<vector<Point2f>> checherboardImageSpacePoints1;
    getChessboardCorners(calibPics2, checherboardImageSpacePoints1,false);
    vector<vector<Point3f>> worldSpaceCornerPoints1;
    coeff2 = Mat::zeros(8,1, 64);
    Mat Error;
    worldSpaceCornerPoints1.resize(calibPics2.size());
    for(int i = 0; i < calibPics2.size(); i++ )
    {
            for(int j = 0; j < chessboardDimensions.height; j++ ){
                for(int k = 0; k < chessboardDimensions.width; k++ ){
                    worldSpaceCornerPoints1[i].push_back(Point3f(k*calibrationSquareDimention, -j*calibrationSquareDimention, 0));
                }
            }
    }
    camMat2 = initCameraMatrix2D(worldSpaceCornerPoints1,checherboardImageSpacePoints1,imageSize,0);
    double err1 = calibrateCamera(worldSpaceCornerPoints1, checherboardImageSpacePoints1, chessboardDimensions, camMat2, coeff2, rvecs2, tvecs2);

    qDebug() << "calibration finished!";
    qDebug() << "rms:" << err1;
    return 0;
}

int GLRender::calibrate3()
{
    Size imageSize;
    Mat rotationMatrix, translationVector, essentialMatrix;

    vector<vector<Point2f>> markedCorners1, rejectedCandidates1;

    cap3 = VideoCapture("http://" + ip3.toStdString() + ":4747/mjpeg");
    if(!cap3.isOpened())
    {
        qDebug() << "IPCAM_3: couldn't establish connection";
        return 1;
    }

    Mat tempPic;
    cap1.read(tempPic);
    imageSize = tempPic.size();

    vector<vector<Point2f>> checherboardImageSpacePoints1;
    getChessboardCorners(calibPics3, checherboardImageSpacePoints1,false);
    vector<vector<Point3f>> worldSpaceCornerPoints1;
    coeff3 = Mat::zeros(8,1, 64);
    Mat Error;
    worldSpaceCornerPoints1.resize(calibPics3.size());
    for(int i = 0; i < calibPics3.size(); i++ )
    {
            for(int j = 0; j < chessboardDimensions.height; j++ ){
                for(int k = 0; k < chessboardDimensions.width; k++ ){
                    worldSpaceCornerPoints1[i].push_back(Point3f(k*calibrationSquareDimention, -j*calibrationSquareDimention, 0));
                }
            }
    }
    camMat3 = initCameraMatrix2D(worldSpaceCornerPoints1,checherboardImageSpacePoints1,imageSize,0);
    Mat rvecs3,tvecs3;
    double err1 = calibrateCamera(worldSpaceCornerPoints1, checherboardImageSpacePoints1, chessboardDimensions, camMat3, coeff3, rvecs3, tvecs3);

    qDebug() << "calibration finished!";
    qDebug() << "rms:" << err1;
    return 0;
}

int GLRender::calibrate()
{
    calibrate1();
    calibrate2();
    calibrate3();

    FileStorage calibration("calibration.yml",FileStorage::WRITE);
    calibration << "camMat1" << camMat1;
    calibration << "coeff1" << coeff1;
    calibration << "camMat2" << camMat2;
    calibration << "coeff2" << coeff2;
    calibration << "camMat3" << camMat3;
    calibration << "coeff3" << coeff3;
    calibration.release();
}

void GLRender::startGlutLoop()
{
    glutLoop = new thread(glutMainLoop);
    glutLoop->detach();
}

void GLRender::getPictures1()
{
    Mat frame1local;
    Mat drawFrame1;

    cap1 = VideoCapture("http://" + ip1.toStdString() + ":4747/mjpeg");
    if(!cap1.isOpened())
    {
        qDebug() << "IPCAM_XY: couldn't establish connection";
        return;
    }
    qDebug()<< "getting images from camera 1" << ip1;
    int i = 0;
    while(i < 5)
    {
        if(!cap1.read(frame1local))
            return;

        rotate(frame1local,frame1local,2);
        flip(frame1local,frame1local,1);

        vector<Vec2f> foundPoints1;
        bool found1 = false;

        found1 = findChessboardCorners(frame1local,chessboardDimensions,foundPoints1,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        frame1local.copyTo(drawFrame1);
        //qDebug()<<found1 << found2;
        if(found1)
        {
            Mat temp1;
            frame1local.copyTo(temp1);
            calibPics1.push_back(temp1);
            qDebug() << "frame found!";
            i++;
        }
    }
    qDebug() << "done!";
}

void GLRender::getPictures2()
{
    Mat frame2local;
    Mat drawFrame2;

    cap2 = VideoCapture("http://" + ip2.toStdString() + ":4747/mjpeg");
    if(!cap2.isOpened())
    {
        qDebug() << "IPCAM_YZ: couldn't establish connection";
        return;
    }
    qDebug()<<"getting images from camera 2" << ip2;
    int i = 0;
    while(i < 5)
    {
        if(!cap2.read(frame2local))
            return;

        rotate(frame2local,frame2local,2);
        flip(frame2local,frame2local,1);

        vector<Vec2f> foundPoints2;
        bool found2 = false;

        found2 = findChessboardCorners(frame2local,chessboardDimensions,foundPoints2,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        frame2local.copyTo(drawFrame2);
        if(found2)
        {
            Mat temp2;
            frame2local.copyTo(temp2);
            calibPics2.push_back(temp2);
            qDebug() << "frame found!";
            i++;
        }
    }
    qDebug() << "done!";

}

void GLRender::getPictures3()
{
    Mat frame2local;
    Mat drawFrame2;

    cap3 = VideoCapture("http://" + ip3.toStdString() + ":4747/mjpeg");
    if(!cap3.isOpened())
    {
        qDebug() << "IPCAM_3: couldn't establish connection";
        return;
    }
    qDebug()<<"getting images from camera 3" << ip3;
    int i = 0;
    while(i < 5)
    {
        if(!cap3.read(frame2local))
            return;

        rotate(frame2local,frame2local,2);
        flip(frame2local,frame2local,1);

        vector<Vec2f> foundPoints2;
        bool found2 = false;

        found2 = findChessboardCorners(frame2local,chessboardDimensions,foundPoints2,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        frame2local.copyTo(drawFrame2);
        if(found2)
        {
            Mat temp2;
            frame2local.copyTo(temp2);
            calibPics3.push_back(temp2);
            qDebug() << "frame found!";
            i++;
        }
    }
    qDebug() << "done!";

}

void GLRender::dumpPictures()
{
    calibPics1.clear();
    calibPics2.clear();
    calibPics3.clear();
}

GLRender::GLRender()
{
    controller = psmove_connect();
    if(controller == NULL)
    {
        qDebug()<<"nie wykryto kontrolera!";
        isSuccess = false;
        return;
    }
    //wheel = imread("color.png", IMREAD_COLOR);
    wheel = imread("color.png", -1);
    if(wheel.empty())
    {
        qDebug()<<"color.png nie istnieje!";
        isSuccess = false;
        return;
    }
    colorWheel = LoadTexture(wheel);

    a=0,b=0,c=0,size=0.01;
    isCalibrated = false;
    erodeIterations = 1;
    dilateIterations = 0;
    erodeBrushSize = 3;
    dilateBrushSize = 3;

    h1 = 600;
    w1 = 450;
    texture = 0;
    texture2 = 0;

    tool = 0;
    toolbelt = 0;
    layer = 0;
    colorMode = false;
    shapeMode = false;
    color = Point3d(1,1,1);

    win1valid = false;
    win2valid = false;
    win3valid = false;

    ticks = 0;
    ticks2 = 0;
    notFoundCount = 0;
    notFoundCount2 = 0;
    found = false;
    found2 = false;

    // <<<< Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    unsigned int type = CV_32F;
    kf = KalmanFilter(stateSize, measSize, contrSize, type);
    state = Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    meas = Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    cv::setIdentity(kf.transitionMatrix);
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;
    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    // <<<< Kalman Filter
    state2 = Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    meas2 = Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    kf2 = KalmanFilter(stateSize, measSize, contrSize, type);
    cv::setIdentity(kf2.transitionMatrix);
    kf2.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf2.measurementMatrix.at<float>(0) = 1.0f;
    kf2.measurementMatrix.at<float>(7) = 1.0f;
    kf2.measurementMatrix.at<float>(16) = 1.0f;
    kf2.measurementMatrix.at<float>(23) = 1.0f;
    kf2.processNoiseCov.at<float>(0) = 1e-2;
    kf2.processNoiseCov.at<float>(7) = 1e-2;
    kf2.processNoiseCov.at<float>(14) = 5.0f;
    kf2.processNoiseCov.at<float>(21) = 5.0f;
    kf2.processNoiseCov.at<float>(28) = 1e-2;
    kf2.processNoiseCov.at<float>(35) = 1e-2;
    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf2.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    init();


    win3 = glutCreateWindow("main");
    glutDisplayFunc(draw3);
    glutReshapeFunc(reshape);
    isSuccess = true;

    loadCalib();

    isCalibrated = true;
    isDrawing = true;
    debug2 = false;
    isArucoCalibrated = false;
    isArucoCalibrated3 = false;
    parametersPtr = &params;
}

GLRender::~GLRender()
{
    if(win1valid)
        win1c();
    if(win3valid)
        win3c();

    kill();
}
