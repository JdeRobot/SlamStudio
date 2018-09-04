/*
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
*/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "winslam.h"
#include "dialogscalatraslarota.h"
#include "transformador2/Transformador.h"
#include "Point3D.h"
class QAction;
class QActionGroup;
class QLabel;
class QMenu;
class GLWidget;
class QDialog;
//class Point3D;

class MainWindow : public QMainWindow

{
    Q_OBJECT

public:
    MainWindow();
    void loadFile(const QString &fileName);
    void setScala(double X, double Y, double Z);
    void setTrasla(double X, double Y, double Z);
    void performModifySequence(double scalaX,double scalaY,double scalaZ, double traslaX,double traslaY, double traslaZ,double rotaX,double rotaY,double rotaZ);
    QWidget *myWinSlam;
    QDialog *dialogScalaTraslaRota;
    Transformador myTransformador;
    char * myInputFileName;
    char * myOutputFileName;
    //Point3D * myScala;
private slots:
    void onOpenFile();
    void onModifySequence();
protected:
    //Winslam myWinSlam;

};

#endif
