#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QDebug>
#include<QKeyEvent>
#include<psmove.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    r = new GLRender();
    if(r->isSuccess == false)
    {
        isSuccess = false;
        return;
    }
    isSuccess = true;
    validIP = false;
    ui->setupUi(this);
    ui->lineEdit->setText("192.168.1.3");
    ui->lineEdit_2->setText("192.168.1.4");
    ui->lineEdit_3->setText("192.168.1.6");
    //ui->lineEdit->setText("192.168.43.48");
    //ui->lineEdit_2->setText("192.168.43.249");
    //qDebug() << "setText";
}
void MainWindow::keyPressEvent( QKeyEvent *e )
{
      if(e->key() == Qt::Key_Space){
            if(r->isDrawing)
                r->isDrawing = false;
            else
                r->isDrawing = true;
      }
}
MainWindow::~MainWindow()
{
    delete r;
    delete ui;
}

void MainWindow::startGlutLoop()
{
    r->startGlutLoop();
}

void MainWindow::on_confirmButton_clicked()
{
    r->ip1 = ui->lineEdit->text();
    r->ip2 = ui->lineEdit_2->text();
    r->ip3 = ui->lineEdit_3->text();
    /*if(r->ipChanged() == 1)
    {
        qDebug() << "IP Change Error";
        validIP = false;
        ui->lineEdit->setStyleSheet("QLineEdit { background: rgb(255,0,0); selection-background-color: rgb(255,120,120); }");
        ui->lineEdit_2->setStyleSheet("QLineEdit { background: rgb(255,0,0); selection-background-color: rgb(255,120,120); }");
        return;
    }
    ui->lineEdit->setStyleSheet("QLineEdit { background: rgb(255,255,255); selection-background-color: rgb(255,120,120); }");
    ui->lineEdit_2->setStyleSheet("QLineEdit { background: rgb(255,255,255); selection-background-color: rgb(255,120,120); }");
    qDebug() << "IP Change Done";
    validIP = true;*/
}
//widocznosc window 1
void MainWindow::on_checkBox_stateChanged(int arg1)
{
    if(!validIP)
    {
        ui->checkBox->setCheckState(Qt::Unchecked);
        return;
    }

    if(arg1 == Qt::Unchecked)
        r->win1c();
    else
        r->win1o();
}
//debug window 1
void MainWindow::on_checkBox_2_stateChanged(int arg1)
{
    r->win1d(arg1);
}
//widocznosc window 2
void MainWindow::on_checkBox_3_stateChanged(int arg1)
{
    if(!validIP)
    {
        ui->checkBox_3->setCheckState(Qt::Unchecked);
        return;
    }

    if(arg1 == Qt::Unchecked)
        r->win3c();
    else
        r->win3o();
}

void MainWindow::on_pushButton_clicked()
{
    r->calibrate();
}

void MainWindow::on_pushButton_2_clicked()
{
    if(r->ipChanged() == 1)
    {
        qDebug() << "IP Change Error";
        validIP = false;
        ui->lineEdit->setStyleSheet("QLineEdit { background: rgb(255,0,0); selection-background-color: rgb(255,120,120); }");
        ui->lineEdit_2->setStyleSheet("QLineEdit { background: rgb(255,0,0); selection-background-color: rgb(255,120,120); }");
        ui->lineEdit_3->setStyleSheet("QLineEdit { background: rgb(255,0,0); selection-background-color: rgb(255,120,120); }");
        return;
    }
    ui->lineEdit->setStyleSheet("QLineEdit { background: rgb(255,255,255); selection-background-color: rgb(255,120,120); }");
    ui->lineEdit_2->setStyleSheet("QLineEdit { background: rgb(255,255,255); selection-background-color: rgb(255,120,120); }");
    ui->lineEdit_3->setStyleSheet("QLineEdit { background: rgb(255,255,255); selection-background-color: rgb(255,120,120); }");
    qDebug() << "IP Change Done";
    validIP = true;
}

void MainWindow::on_doubleSpinBox_4_valueChanged(double arg1)
{
    r->size = arg1;
}

void MainWindow::on_pushButton_3_clicked()
{
    r->getPictures1();
}

void MainWindow::on_pushButton_4_clicked()
{
    r->getPictures2();
}

void MainWindow::on_pushButton_5_clicked()
{
    r->getPictures3();
}

void MainWindow::on_pushButton_6_clicked()
{
    r->dumpPictures();
}
