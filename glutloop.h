#ifndef GLUTLOOP_H
#define GLUTLOOP_H

#include "GL/freeglut.h"
#include "GL/gl.h"
#include "GL/glu.h"
#include <vector>
#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv/highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <chrono>
#include <thread>
#include <mutex>
#include <unistd.h>

using namespace std;
using namespace cv;
using namespace chrono;

#define ERODE_ITERATIONS 1
#define DILATE_ITERATIONS 0
#define B 1000
#define G 1000

class glutLoop
{
    //przyklad uzycia ukladanie klockow kolorowych tetrisowych na kartce
    //g++ -std=c++11 -pthread main.cc -lglut -lGL -lGLU `pkg-config --cflags --libs opencv`



    void init();
    void draw();
    void draw2();
    int main(int argc, char **argv);

    void reshape(GLsizei width, GLsizei height);
    void vidCap1();
    void vidCap2();
    void keyboardFunkc(unsigned char key,int x,int y);
    void kill();
    void lights(int i);
    void orthogonalStart();
    void orthogonalEnd();
    void background(GLuint t);
    GLuint LoadTexture( Mat m );

    glutLoop();

    public:

    glutLoop(bool xy,bool yz,string ip1, string ip2, int argc, char **argv);
};

#endif // GLUTLOOP_H
