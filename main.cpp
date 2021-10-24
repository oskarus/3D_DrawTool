#include "mainwindow.h"
#include <QApplication>
#include "GL/freeglut.h"
#include "GL/gl.h"
#include "GL/glu.h"
#include <QDebug>
#include <thread>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    MainWindow w;
    if(!w.isSuccess)
    {
        qDebug() << "initialization failed, closing";
    }
    glutHideWindow();
    //glutMainLoop();
    //w.startGlutLoop();
    w.show();
    while(1)
    {
        glutMainLoopEvent();
        a.processEvents();
    }
    //return a.exec();
}
