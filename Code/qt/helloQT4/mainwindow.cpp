/*
 * #include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
*/

#include "mainwindow.h"
#include "window.h"
#include "winslam.h"
#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include <QtWidgets>
#include <QDialog>
#include "Eigen/Dense"
#include "transformador2/Transformador.h"
//#include "transformador2/Point3D.h"
//#include <QGLWidget>


MainWindow::MainWindow()
{
    QMenuBar *menuBar = new QMenuBar;
    QMenu *menuWindow = menuBar->addMenu(tr("&File"));
    QAction *openFile = new QAction(menuWindow);
    openFile->setText(tr("Open"));
    menuWindow->addAction(openFile);
    connect(openFile, &QAction::triggered, this, &MainWindow::onOpenFile);

    QMenu *menuSequence = menuBar->addMenu(tr("&Sequence Manager"));
    QAction *modifySequence = new QAction(menuWindow);
    modifySequence->setText(tr("Modify sequence"));
    menuSequence->addAction(modifySequence);
    connect(modifySequence, &QAction::triggered, this, &MainWindow::onModifySequence);
    setMenuBar(menuBar);

    setMenuBar(menuBar);

    if (!centralWidget())
        //setCentralWidget(new Window(this));
        myWinSlam=(QWidget*)(new Winslam(this));
        setCentralWidget(myWinSlam);
    //onAddNew();

}

void MainWindow::onOpenFile()
{
    /*if (!centralWidget())
        setCentralWidget(new Window(this));
    else
        QMessageBox::information(0, tr("Cannot add new window"), tr("Already occupied. Undock first."));
    */
    //infoLabel->setText(tr("Invoked <b>File|Open</b>"));
    QString fileName = QFileDialog::getOpenFileName(this);
    std::string fname = fileName.toStdString();
    char * cstr = new char [fname.size()+1];
    strcpy( cstr, fname.c_str() );
    myInputFileName= cstr;
    if (!fileName.isEmpty())
        loadFile(fileName);
}

void MainWindow::onModifySequence()
{
     std::cout<< "onModifySequence" <<std::endl;
    dialogScalaTraslaRota =(QDialog*) (new DialogScalaTraslaRota (this));
    dialogScalaTraslaRota->show();
    std::cout<< "FIN onModifySequence" <<std::endl;
}
void MainWindow::loadFile(const QString &fileName)
{
    QFile file(fileName);
    int maxLine = 3000;
    Eigen::MatrixXd readingA (maxLine,3);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, tr("Application"),
                             tr("Cannot read file %1:\n%2.")
                             .arg(QDir::toNativeSeparators(fileName), file.errorString()));
        return;
    } else{
        std::cout<< "leyendo archivo" <<std::endl;
        std::ifstream infileA( fileName.toStdString());
        double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
        int contLin=0;

        while ( (infileA >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
                    std::cout <<"contLin="<<contLin<<std::endl;
                    readingA.row(contLin)<< rx,ry,rz;
                    std::cout <<"contLin="<<contLin<<std::endl;
                    contLin ++;


        }
        infileA.close();

        std::cout <<"fin leyendo Archivo"<<std::endl;
    }

    QTextStream in(&file);
#ifndef QT_NO_CURSOR
    QApplication::setOverrideCursor(Qt::WaitCursor);
#endif
    //textEdit->setPlainText(in.readAll());
#ifndef QT_NO_CURSOR
    QApplication::restoreOverrideCursor();
#endif

    //setCurrentFile(fileName);
    statusBar()->showMessage(tr("File loaded"), 2000);
    ((Winslam*)(myWinSlam))->setDataView(readingA);
}

void MainWindow::setScala(double X, double Y, double Z){
     std::cout<< "MainWindow.setScala" <<std::endl;
    ((Winslam*)(myWinSlam))->setScala(X,Y,Z);
}

void MainWindow::setTrasla(double X, double Y, double Z){
    ((Winslam*)(myWinSlam))->setTrasla(X,Y,Z);
    performModifySequence(X,Y,Z, X,Y, Z,0,0,0);
}

void MainWindow::performModifySequence(double scalaX,double scalaY,double scalaZ, double traslaX,double traslaY, double traslaZ,double rotaX,double rotaY,double rotaZ){
     int maxLine = 3000;
     Point3D  myScala;
     myScala.setXYZ(scalaX,scalaY,scalaZ);

     Point3D miTraslacion ;
     miTraslacion.setXYZ(traslaX,traslaY,traslaZ);

     myOutputFileName="miSalidaContaminadaQT.txt";
     myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,0,'X',0,0,0.0);

     // Reading input file B, with new dataset contaminated
     std::ifstream infileB( "miSalidaContaminadaQT.txt" );
     Eigen::MatrixXd readingB (maxLine,3);
     double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
     int contLin=0;

     while ( (infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
                         readingB.row(contLin)<<rx,ry,rz;

                         contLin ++;

      }
     infileB.close();
     ((Winslam*)(myWinSlam))->setContaminatedDataView(readingB);

}


