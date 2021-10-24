#include "glutloop.h"

void glutLoop::lights(int i)
{
    if(i > 0)
    {
        glShadeModel(GL_SMOOTH);
        glEnable(GL_LIGHTING);
        glEnable( GL_LIGHT0 );
        // Create light components
        GLfloat ambientLight[] = { 0.4, 0.4, 0.4, 1.0 };
        GLfloat diffuseLight[] = { 0.7, 0.7, 0.7, 1.0 };
        GLfloat specularLight[] = { 1, 1, 1, 1.0 };
        GLfloat position[] = { 1,1, 0, 10 };
        GLfloat qa[] = {0};
        GLfloat la[] = {0};
        GLfloat ca[] = {1};
        GLfloat se[] = {16};
        GLfloat sc[] = {30};
        GLfloat sd[] = {0,0,-1};

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

void glutLoop::orthogonalStart()
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(-w1/2, w1/2, -h1/2, h1/2);
    glMatrixMode(GL_MODELVIEW);
}

void glutLoop::orthogonalEnd()
{
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void glutLoop::background(GLuint t)
{
    glDisable(GL_DEPTH_TEST);
    //lights(-1);
    glBindTexture( GL_TEXTURE_2D, t );

    orthogonalStart();

    // texture width/height
    const int iw = 500;
    const int ih = 500;

    glPushMatrix();
    glTranslatef( -iw/2, -ih/2, 0 );
    glBegin(GL_QUADS);
        glTexCoord2i(0,0); glVertex2i(0, 0);
        glTexCoord2i(1,0); glVertex2i(iw, 0);
        glTexCoord2i(1,1); glVertex2i(iw, ih);
        glTexCoord2i(0,1); glVertex2i(0, ih);
    glEnd();
    glPopMatrix();

    orthogonalEnd();
    glEnable(GL_DEPTH_TEST);
    //lights(1);
}

GLuint glutLoop::LoadTexture(Mat m)
{
    GLuint texture;

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

      // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


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

void glutLoop::init()
{
    //adres ip: "http://192.168.1.1:4747/mjpeg"
    cap1 = VideoCapture(ip1);
    if(!cap1.isOpened())
        return;

    cap2 = VideoCapture(ip2);
    if(!cap2.isOpened())
        return;

    glClearColor(0,0,0,1);
    glClearDepth(1);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    //lights(1);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void glutLoop::draw()
{
    chrono::high_resolution_clock::time_point t0 = chrono::high_resolution_clock::now();
    Mat frame1local,frame2local;
    Mat calc;
    Mat tex;

    mut1.lock();
    frame1local = frame1;
    mut1.unlock();
    mut2.lock();
    frame2local = frame2;
    mut2.unlock();

    chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();

    //image2 = frame2;
    inRange(frame1,Scalar(210, 210, 210),Scalar(255, 255, 255),calc);
    //cvtColor(frame,calc,COLOR_RGB2GRAY);
    //cvtColor(frame,edges,COLOR_RGB2BIN);
    //GaussianBlur(calc,calc,Size(15,15),5,5);

    //threshold(frame, calc, 235, 255, CV_THRESH_BINARY);

    dilate(calc,calc , 0, Point(-1, -1), DILATE_ITERATIONS, 1, 1);
    erode(calc, calc, getStructuringElement(MORPH_RECT,Size(3,3),Point(1,1)), Point(-1, -1), ERODE_ITERATIONS, 1, 1);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int largest_area=0;
    int largest_contour_index=0;
    Rect bounding_rect;
    findContours(calc, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    for( int i = 0; i < contours.size(); i++ ) // iterate through each contour.
    {
        double a=contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area)
        {
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
            bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
        }
    }

    chrono::high_resolution_clock::time_point t3 = chrono::high_resolution_clock::now();

    rectangle(frame1, bounding_rect, Scalar(255, 0, 0), 1, 8, 0 );
    tex = frame1;
    texture  = LoadTexture(tex);

    Point center1 = (bounding_rect.br() + bounding_rect.tl())*0.5;

    chrono::high_resolution_clock::time_point t4 = chrono::high_resolution_clock::now();

    contours.clear();
    hierarchy.clear();
    largest_area=0;
    largest_contour_index=0;

    inRange(frame2,Scalar(210, 210, 210),Scalar(255, 255, 255),calc);
    //GaussianBlur(calc,calc,Size(15,15),5,5);
    dilate(calc,calc , 0, Point(-1, -1), DILATE_ITERATIONS, 1, 1);
    erode(calc, calc, getStructuringElement(MORPH_RECT,Size(3,3),Point(1,1)), Point(-1, -1), ERODE_ITERATIONS, 1, 1);
    findContours(calc, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a=contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
            bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
        }
    }

    chrono::high_resolution_clock::time_point t5 = chrono::high_resolution_clock::now();

    //rectangle(image1, bounding_rect, Scalar(255, 0, 0), 1, 8, 0 );

    Point center2 = (bounding_rect.br() + bounding_rect.tl())*0.5;
    //cout << center1<< endl;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable( GL_TEXTURE_2D );

    chrono::high_resolution_clock::time_point t = chrono::high_resolution_clock::now();
        background(texture);
        //gluLookAt (0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    //glTranslatef(0.2,0,0);

    chrono::high_resolution_clock::time_point t6 = chrono::high_resolution_clock::now();

    //drawQube(0.5,0,-5,0.3,0,0,0);
    float x,y,z;
    x = (((float)center1.x - (frame1.cols/2.0)) / (frame1.cols/2.0));
    y = (((float)center1.y - (frame1.rows/2.0)) / (frame1.rows/2.0));
    z = (( -1*(float)(center2.x) / (float)frame2.cols ) ) - 0.3;
    //z = ((( (float)(center2.x) / (float)frame2.cols )-1)*3 )-2;
    //z = ((( (float)(center2.x) / (float)frame2.cols )-3);
    //z = - 6;
    //cout << x << y << z << endl;
    glTranslatef(x,y,z);
    glutSolidCube(0.2);
    //glTranslatef(-0.5,0,-5);
    //glutSolidSphere(0.5,40,40);

    glutSwapBuffers();

    chrono::high_resolution_clock::time_point t7 = chrono::high_resolution_clock::now();
    auto full = duration_cast<microseconds>( t7 - t0 ).count(),getvid= duration_cast<microseconds>( t1 - t0 ).count()
    ,contours1= duration_cast<microseconds>( t3 - t1 ).count(),lTexture= duration_cast<microseconds>( t4 - t3 ).count()
    ,contours2= duration_cast<microseconds>( t5 - t4 ).count(),dTexture= duration_cast<microseconds>( t6 - t ).count()
    ,render3d= duration_cast<microseconds>( t7 - t6 ).count();

    cout<<"videoCap:"<< (getvid*100)/full <<"% "<<endl
        <<"contoursXY:"<< (contours1*100)/full <<"% "<<endl
        <<"LoadTexture:"<< (lTexture*100)/full <<"% "<<endl
        <<"contoursZ:"<< (contours2*100)/full <<"% "<<endl
        <<"renderTexture:"<< (dTexture*100)/full <<"% "<<endl
        <<"render3d:"<< (render3d*100)/full <<"% "<<endl<<endl;

    glutPostRedisplay();
}

void glutLoop::draw2()
{
    Mat frame2;
    Mat calc;
    Mat tex;
    cap2 >> frame2;
    rotate(frame2,frame2,2);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int largest_area=0;
    int largest_contour_index=0;
    Rect bounding_rect;

    inRange(frame2,Scalar(210, 210, 210),Scalar(255, 255, 255),calc);
    //GaussianBlur(calc,calc,Size(15,15),5,5);
    dilate(calc,calc , 0, Point(-1, -1), DILATE_ITERATIONS, 1, 1);
    erode(calc, calc, getStructuringElement(MORPH_RECT,Size(3,3),Point(1,1)), Point(-1, -1), ERODE_ITERATIONS, 1, 1);
    findContours(calc, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a=contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
            bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
        }
    }
    rectangle(frame2, bounding_rect, Scalar(255, 0, 0), 1, 8, 0 );

    texture2 = LoadTexture(frame2);

    glEnable( GL_TEXTURE_2D );

  background(texture2);
  //cout << "dziala" << endl;
    glutSwapBuffers();
    glutPostRedisplay();
}

void glutLoop::reshape(GLsizei width, GLsizei height)
{
    // GLsizei for non-negative integer
  // Compute aspect ratio of the new window
  if (height == 0) height = 1;                // To prevent divide by 0
  GLfloat aspect = (GLfloat)width / (GLfloat)height;
    w1 = width;
  h1 = height;

  // Set the viewport to cover the new window
  glViewport(0, 0, width, height);

  // Set the aspect ratio of the clipping volume to match the viewport
  glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
  glLoadIdentity();             // Reset
  // Enable perspective projection with fovy, aspect, zNear and zFar
  gluPerspective(45, aspect, 0.1, 100);
}

void glutLoop::vidCap1()
{
    Mat frame;
    while(1)
    {
        cap1 >> frame;
        if(mut1.try_lock() == false)
        {
            usleep(5000);
            continue;
        }

        rotate(frame,frame,2);
        frame1 = frame;
        mut1.unlock();
    }
}

void glutLoop::vidCap2()
{
    Mat frame;
    while(1)
    {
        cap2 >> frame;
        if(mut2.try_lock() == false){
            usleep(5000);
            continue;
        }

        rotate(frame,frame,2);
        frame2 = frame;
        mut2.unlock();
    }
}

void glutLoop::keyboardFunkc(unsigned char key, int x, int y)
{
    switch(key){
        case 27: kill(); break;
    };
}

void glutLoop::kill()
{
    //zabij watki
    threads.at(0).~thread();
    threads.at(1).~thread();
    //zamknij okna

}

extern void keyboardFunkcEx( unsigned char key, int x, int y )
{
    return this->keyboardFunkc(key, x, y);
}

int glutLoop::main(int argc, char **argv)
{

    int window,win2;
    glutInit(&argc, argv);

    init();
    thread t1(&glutLoop::vidCap1),t2(&glutLoop::vidCap2);
    t1.detach();
    t2.detach();
    threads.push_back(t1);
    threads.push_back(t2);

    glutKeyboardFunc(keyboardFunkc);
    glutInitDisplayMode(GLUT_DOUBLE);
    if(xyView)
    {
        glutInitWindowSize(600, 600);
        glutInitWindowPosition(100, 100);
        window = glutCreateWindow("Render3D_XY");
        glutDisplayFunc(draw);
        glutReshapeFunc(reshape);
    }
    if(yzView)
    {
        glutInitWindowSize(600, 600);
        glutInitWindowPosition(800, 100);
        win2 = glutCreateWindow("Render3D_Z");
        glutDisplayFunc(draw2);
        glutReshapeFunc(reshape);
    }

  glutMainLoop();
  return 0;
}

glutLoop::glutLoop()
{

}

glutLoop::glutLoop(bool xy, bool yz, string ip1, string ip2, int argc, char **argv)
{
    this->xyView = xy;
    this->yzView = yz;
    this->ip1 = ip1;
    this->ip2 = ip2;

    this->main(argc,argv);
}

