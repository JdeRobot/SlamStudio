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
#include <string>
#include "winslam.h"
#include "datadialogscalatraslarota.h"
#include "datadialogshowestimated.h"
#include "dialogscalatraslarota.h"
#include "dialogshowestimated.h"
#include "dialogmessage.h"
#include "datadialogparameters.h"
#include "dialogparameters.h"
#include "transformador2/Transformador.h"
#include "Registrador/Registrador.h"
#include "GeneratorPCA/GeneratorPCA.h"
#include "ModuloEscala/FindScala.h"
#include "AjusteTiempo/AjusteTiempo.h"
#include "Interpolator/Interpolator.h"
#include "Point3D.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
class QAction;
class QActionGroup;
class QLabel;
class QMenu;
class GLWidget;
class QDialog;
class QProgressBar;
//class Point3D;

class MainWindow : public QMainWindow

{
    Q_OBJECT

public:
    MainWindow();
    void loadFile(const QString &fileName, MatrixXd &dataset, int dataSet_A_B);
    void setScala(double X, double Y, double Z);
    void setTrasla(double X, double Y, double Z);
    void performModifySequence(double scalaX,double scalaY,double scalaZ, double traslaX,double traslaY, double traslaZ,double rotaX,double rotaY,double rotaZ,double gNoise,double cNoise, double timeOffset,int pcaIndex,double frequency);
    void setTextOnStatusBar(QString aText);
    void setProperties();
//    void setMaxLines(int numLines);
//    void setOffsetWindow(double offsetWindow);
//    void setOffsetStep(double offsetStep);
//    int getMaxLines();
//    double getOffsetWindow();
//    double getOffsetStep();
    void cleanDataSets(); // this method clean the datasets, datasetA,datasetB and datasetEstimated
    QWidget *myWinSlam;
    QLabel *statusLabel;
    QProgressBar *statusProgressBar;



    //DIALOG that shows parameters of transformation to be performed , scale, traslation , rotation, gaussian and cosmic noise
    DialogScalaTraslaRota *dialogScalaTraslaRota;
    DataDialogScalaTraslaRota* dataDialogScalaTraslaRota;

    //DIALOG that shows parameters of ESTIMATED transformations, scale, traslation, rotation
    DialogShowEstimated *dialogShowEstimated;
    DataDialogShowEstimated *dataDialogShowEstimated;

    //DIALOG that shows CONFIGURATION parameters
    //DialogConfiguration *dialogConfiguration;
    DataDialogParameters *dataDialogParameters;
    DialogParameters *dialogParameters;



    DialogMessage* dialogMessage;

    //PARAMETERS
    //int MAXLINES=40000;
    //double offsetWindow=5;
    //double offsetStep=0.01;


    //============================== Model Objects to estimate transformation
    Transformador myTransformador; //Perform transformations (Scale,Rotation,Traslation,Gaussian and Cosmic Noise ) over dataset
    Registrador myRegistrador;  //Estimate Rotation and Traslation over a dataset
    GeneratorPCA myGeneratorPCA;//Calculate PCA over a dataset
    FindScala myFindScala; // to find the scala
    AjusteTiempo myFindOffset; //to find timeOffset
    Interpolator myInterpolator;// to interpolate
    char * myInputFileName;
    char * myOutputFileName;
    //==============================

    //============================== Matrix that will store the data model
    Eigen::MatrixXd readingA;
    Eigen::MatrixXd readingB;
    Eigen::MatrixXd readingAbkp;
    Eigen::MatrixXd readingBbkp;
    Eigen::MatrixXd dataAquaternion;
    Eigen::MatrixXd dataBquaternion;
    Eigen::MatrixXd dataQuaternionEstimated;
    Eigen::MatrixXd dataEstimated;
    Eigen::MatrixXd rotationEstimated;
    Eigen::MatrixXd traslationEstimated;
    Eigen::Quaterniond quat_FromRotEstimated; //quaternion from rotation estimated
    Eigen::VectorXd timeA; //[10000];
    Eigen::VectorXd timeB; //[10000];
    //double timeA [10000];
    //double timeB [10000];

    //===============================
    double timeOffset;
    double timeOffsetEstimated;
    double rMax;
    double frequency=-1;
    int ftype=-1;
    bool dataSetB_isLoaded_NotTransformed=false;//this variable indicate if dataset B is loaded from another file or if is the result of transforming dataSet A

    //CONFIGURATION PROPERTIES
    //double interpolationStep=0.0;
    //int maxLineInputFile=0.0;
    //int maxLineInterpolation=0.0;
    //double offsetLimit=0.0;



    //Point3D * myScala;
private slots:
    void onOpenFileA();
    void onOpenFileB();
    void onModifySequence();
    void onEstimateSequence(int way); // way==0 means A to B .  way==1 means B to A
    void onEstimateSequenceAtoB();
    void onEstimateSequenceBtoA();
    void onExit();
    void onSetDots();//Menu View:indicate that dataset will be displayed as 3d points
    void onSetLines();//Menu View:indicate that dataset will be displayed as 3d lines
    void onViewJustEstimated();//Menu View; indicate that only estimated dataset will be shown
    void onModifyParameters();//Menu View: Shows several configuration parameters to be changed. Example:Max Number of Lines for dataset
protected:
    //Winslam myWinSlam;

};

#endif
