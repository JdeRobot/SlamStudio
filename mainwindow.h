#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <QtWidgets>
#include <QDialog>
#include <QProgressBar>
#include <iostream>
#include <fstream>
#include <string>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Properties/ReaderProperties.h"
#include "configuration.h"
#include "Point3D.h"
#include "Statistics/Statistics.h"
#include "datadialogshowestimated.h"
#include "dialogscalatraslarota.h"
#include "winslam.h"
#include "dialogshowestimated.h"
#include "datadialogparameters.h"
#include "dialogparameters.h"
#include "dialogmessage.h"
#include "transformation/Transformador.h"
#include "Registrador/Registrador.h"
#include "GeneratorPCA/GeneratorPCA.h"
#include "ModuloEscala/FindScala.h"

class QAction;
class QLabel;
class QMenu;
class GLWidget;
class QDialog;
class QProgressBar;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void loadFile(const QString &fileName, Eigen::MatrixXd &dataset, int dataSet_A_B);
    void performModifySequence(double scalaX,double scalaY,double scalaZ, double traslaX,double traslaY, double traslaZ,double rotaX,double rotaY,double rotaZ, int gNoise, int cNoise, double timeOffset,int pcaIndex,double frequency, double gnoise_value);
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


    //============================== Model Objects to estimate transformation
    Transformador myTransformador; //Perform transformations (Scale,Rotation,Traslation,Gaussian and Cosmic Noise ) over dataset
    Registrador myRegistrador;  //Estimate Rotation and Traslation over a dataset
    GeneratorPCA myGeneratorPCA;//Calculate PCA over a dataset
    FindScala myFindScala; // to find the scala
    Interpolator myInterpolator;// to interpolate
    char * myInputFileName;
    char * myOutputFileName;
    //==============================

    //============================== Matrix that will store the data model
    Eigen::MatrixXd readingA;
    Eigen::MatrixXd readingB;
    Eigen::MatrixXd readingA_xyz;
    Eigen::MatrixXd readingB_xyz;
    Eigen::MatrixXd readingAquaternion;
    Eigen::MatrixXd readingBquaternion;
    Eigen::MatrixXd dataQuaternionEstimated;
    Eigen::MatrixXd dataEstimated;
    Eigen::MatrixXd rotationEstimated;
    Eigen::MatrixXd traslationEstimated;
    Eigen::VectorXd timeA; //[10000];
    Eigen::VectorXd timeB; //[10000];


    //===============================
    double timeOffset;
    double timeOffsetEstimated;
    double frequency=-1;
    int ftype=-1;
    bool dataSetB_isLoaded_NotTransformed=false;//this variable indicate if dataset B is loaded from another file or if is the result of transforming dataSet A



private:
    Ui::MainWindow *ui;
    void onOpenFileA();
    void onOpenFileB();
    void onModifySequence();
    void onEstimateSequence(int way,bool RANSAC); // way==0 means A to B .  way==1 means B to A
    void onEstimateSequenceAtoB();
    void onEstimateSequenceBtoA();
    void onEstimateSequenceAtoB_RANSAC();
    void onEstimateSequenceBtoA_RANSAC();
    void onExit();
    void onSetDots();//Menu View:indicate that dataset will be displayed as 3d points
    void onSetLines();//Menu View:indicate that dataset will be displayed as 3d lines
    void onViewJustEstimated();//Menu View; indicate that only estimated dataset will be shown
    void onModifyParameters();//Menu View: Shows several configuration parameters to be changed. Example:Max Number of Lines for dataset

};

#endif // MAINWINDOW_H
