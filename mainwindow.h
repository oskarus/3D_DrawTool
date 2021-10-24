#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "render.h"
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool isSuccess;
    bool validIP;
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void startGlutLoop();

private slots:

    void on_confirmButton_clicked();

    void on_checkBox_stateChanged(int arg1);

    void on_checkBox_2_stateChanged(int arg1);

    void on_checkBox_3_stateChanged(int arg1);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_doubleSpinBox_4_valueChanged(double arg1);

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void keyPressEvent(QKeyEvent *event);

private:
    Ui::MainWindow *ui;
    int ei,es,di,ds;
    GLRender *r;
};

#endif // MAINWINDOW_H
