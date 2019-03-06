
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
#include "Statistics/Statistics.h"
//#include "transformador2/Point3D.h"
//#include <QGLWidget>


MainWindow::MainWindow()
{
    int maxLine = 3000;
    Eigen::MatrixXd newMatrixA (maxLine,3);
    readingA = newMatrixA;
    Eigen::MatrixXd newMatrixB (maxLine,3);
    readingB = newMatrixB;
    Eigen::MatrixXd newMatrixEstimated (maxLine,3);
    dataEstimated = newMatrixEstimated;
    QMenuBar *menuBar = new QMenuBar;
    QMenu *menuWindow = menuBar->addMenu(tr("&File"));
    QAction *openFile = new QAction(menuWindow);
    openFile->setText(tr("Open"));
    menuWindow->addAction(openFile);
    connect(openFile, &QAction::triggered, this, &MainWindow::onOpenFile);
    QAction *exitApp = new QAction(menuWindow);
    exitApp->setText(tr("Exit"));
    menuWindow->addAction(exitApp);
    connect(exitApp, &QAction::triggered, this, &MainWindow::onExit);

    QMenu *menuSequence = menuBar->addMenu(tr("&Sequence Manager"));
    QAction *modifySequence = new QAction(menuWindow);
    modifySequence->setText(tr("Modify sequence"));
    menuSequence->addAction(modifySequence);
    connect(modifySequence, &QAction::triggered, this, &MainWindow::onModifySequence);

    QMenu *menuEstimate = menuBar->addMenu(tr("&Estimator"));
    QAction *estimateSequenceAtoB = new QAction(menuWindow);
    estimateSequenceAtoB->setText(tr("Estimate sequence A to B"));
    menuEstimate->addAction(estimateSequenceAtoB);
    connect(estimateSequenceAtoB, &QAction::triggered, this, &MainWindow::onEstimateSequenceAtoB);

    QAction *estimateSequenceBtoA = new QAction(menuWindow);
    estimateSequenceBtoA->setText(tr("Estimate sequence B to A"));
    menuEstimate->addAction(estimateSequenceBtoA);
    connect(estimateSequenceBtoA, &QAction::triggered, this, &MainWindow::onEstimateSequenceBtoA);



    QMenu *menuView = menuBar->addMenu(tr("&View"));
    QAction *setDots = new QAction(menuWindow);
    setDots->setText(tr("3d dots"));
    menuView->addAction(setDots);
    connect(setDots, &QAction::triggered, this, &MainWindow::onSetDots);
    QAction *setLines = new QAction(menuWindow);
    setLines->setText(tr("3d lines"));
    menuView->addAction(setLines);
    connect(setLines, &QAction::triggered, this, &MainWindow::onSetLines);
    QAction *setViewJustEstimated = new QAction(menuWindow);
    setViewJustEstimated->setText(tr("show only data estimated"));
    menuView->addAction(setViewJustEstimated);
    connect(setViewJustEstimated, &QAction::triggered, this, &MainWindow::onViewJustEstimated);


    setMenuBar(menuBar);

    //setMenuBar(menuBar);

    if (!centralWidget())
        //setCentralWidget(new Window(this));
        myWinSlam=(QWidget*)(new Winslam(this));
        setCentralWidget(myWinSlam);
    //onAddNew();
    dataDialogScalaTraslaRota= new DataDialogScalaTraslaRota(1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0.005);

}

void MainWindow::onExit()
{
    this->close();
}
void MainWindow::onSetDots()
{
    ((Winslam*)(myWinSlam))->setDots();
}
void MainWindow::onSetLines()
{
    ((Winslam*)(myWinSlam))->setLines();
}
void MainWindow::onViewJustEstimated()
{
    ((Winslam*)(myWinSlam))->setViewJustEstimated();
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
//================================================================================================================

void MainWindow::onModifySequence()
{
     std::cout<< "onModifySequence" <<std::endl;
    //dialogScalaTraslaRota =(QDialog*) (new DialogScalaTraslaRota (this));
    dialogScalaTraslaRota =(new DialogScalaTraslaRota (this));
    //dialogScalaTraslaRota->setDataDialog(new DataDialogScalaTraslaRota(1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0));
    dialogScalaTraslaRota->show();
    dialogScalaTraslaRota->setDataDialog(dataDialogScalaTraslaRota);
    if (dataDialogScalaTraslaRota->getFrequencyType()==0){
        dialogScalaTraslaRota->onPressMaxFrequency();
    } else if (dataDialogScalaTraslaRota->getFrequencyType()==1){
        dialogScalaTraslaRota->onPressMinFrequency();
    }else if (dataDialogScalaTraslaRota->getFrequencyType()==2){
        dialogScalaTraslaRota->onPressCustomizedFrequency();
    }
    dialogScalaTraslaRota->update();
    dataDialogScalaTraslaRota= dialogScalaTraslaRota->getDataDialog();
    std::cout<<"onModifySequence.dataDialogScalaTraslaRota->getCosmicNoiseDeviation()"<<dataDialogScalaTraslaRota->getCosmicNoiseDeviation();
    std::cout<< "FIN onModifySequence" <<std::endl;
}
//================================================================================================================

void MainWindow::onEstimateSequence(int way){//Estimate transformations from dataset A to dataset B or dataset B to dataset A
    //frequency=0.005;
    float rMax=0.0;
    std::cout<<"onEstimateSequenceAtoB frequency="<<frequency<<std::endl;
    //myRegistrador.rigid_transform_3D()
    readingA=readingAbkp.block(0,0,readingAbkp.rows(),3);//to use with Scale and PCA
    readingB=readingBbkp.block(0,0,readingBbkp.rows(),3);

    int contLin = readingB.rows();
    std::cout <<"MainWindow::onEstimateSequence contLin="<<contLin<<std::endl;
    //Begin Try to estimate Scala using PCA
    //=========================================
    MatrixXd AA,BB,AApca,BBpca (contLin,3);
    MatrixXd pcaA,pcaB;
    //B = readingB.block(0,0,contLin,3);
    AA = readingA.block(0,0,readingA.rows(),3);//to use with Scale and PCA
    BB = readingB.block(0,0,readingB.rows(),3);//to use with Scale and PCA

    myGeneratorPCA.calculatePCAbySVD(0,AA, AApca, pcaA);//A is converted to PCA. Important,inside this function also is calculated A.rowwise() - A.colwise().mean(). A is converted to a newA
    myGeneratorPCA.calculatePCAbySVD(0,BB, BBpca, pcaB);//B is converted to PCA. Important,inside this function also is calculated B.rowwise() - B.colwise().mean(). B is converted to a newB
    myGeneratorPCA.setPcaA(pcaA);
    myGeneratorPCA.setPcaB(pcaB);

    Vector3d myScalaSVD = myFindScala.getScalaSVD(AApca,BBpca);
    std::cout <<"MainWindow::onEstimateSequence myScalaSVD="<<myScalaSVD<<std::endl;

    // End Try to estimate Scala using PCA
    //==========================================

    // Try to find Time offset
    //==========================================
    double offsetEstimated = 0;
    if (timeOffset > 0) {
            int maxLine = readingA.rows();
            int interval = 50;
            double anOffset = timeOffset;
            double correlationValue = 0;
            //double correlationValue = myFindOffset.calculateOffsetXYZ( maxLine,interval, anOffset,offsetEstimated, AApca,  BBpca);//ok
            //timeOffsetEstimated= offsetEstimated;
            //std::cout<<"timeOffsetEstimated="<<timeOffsetEstimated<<std::endl;

        // Calculate interpolation with AApca and BBpca
        // But AApca and BBpca has only 3 coordinates, needs to add the 4th coordinate , which is TIME

        //anOffset=0;
        MatrixXd tAApca(contLin,4),tBBpca (contLin,4);// store aapca and bbpca with time
        for (int i =0; i < contLin; i ++){
            std::cout<<"inside the for i="<<i<<std::endl;
            //tAApca(i,0)=AA(i,0);
            tAApca(i,0)=timeA[i];
            tAApca(i,1)=AApca(i,0);
            tAApca(i,2)=AApca(i,1);
            tAApca(i,3)=AApca(i,2);
            //tBBpca(i,0)=BB(i,0);
            tBBpca(i,0)=timeB[i];
            tBBpca(i,1)=BBpca(i,0);
            tBBpca(i,2)=BBpca(i,1);
            tBBpca(i,3)=BBpca(i,2);

        }
        for (int t=0; t<10; t++){
               std::cout<<"================================================================timeA["<<t<<"]="<<tAApca(t,0)<<"  timeB["<<t<<"]="<<tBBpca(t,0)<<std::endl;
        }

        //rMax=myInterpolator.calculateOffsetWithInterpolation(maxLine,interval, anOffset,offsetEstimated, tAApca,  tBBpca);
        //rMax=myInterpolator.calculateOffsetWithInterpolation2( tAApca,  tBBpca);
        //timeOffsetEstimated= offsetEstimated;

        timeOffsetEstimated=myInterpolator.calculateOffsetWithInterpolation2( tAApca,  tBBpca,rMax);//Bueno
        timeOffsetEstimated= int(timeOffsetEstimated*1000)/1000.0;
        //timeOffsetEstimated=myInterpolator.calculateOffsetWithInterpolation3( tAApca,  tBBpca);
        std::cout<<"timeOffsetEstimated2="<<timeOffsetEstimated<<std::endl;
        std::cout<<"timeOffset EXPECTED="<<timeOffset<<std::endl;
        std::cout<<"rMax="<<rMax<<std::endl;

        //Apply estimated offset over B
        for (int i=0; i< tBBpca.rows(); i++){
            Vector4d myNewVector(((tBBpca.row(i))(0) + offsetEstimated),(tBBpca.row(i))(1),(tBBpca.row(i))(2),(tBBpca.row(i))(3));
            //contAddedValues++;



            tBBpca.row(i)<<myNewVector.transpose();
         }
         //myInterpolator.interpolateSerieToFrequency2(frequency, tAApca);
        //Check if the time is aligned for A and B
         for (int t=0; t<10; t++){
                std::cout<<"================================================================timeA["<<t<<"]="<<tAApca(t,0)<<"  timeB["<<t<<"]="<<tBBpca(t,0)<<std::endl;
         }

    } else {
         timeOffsetEstimated=0;
         rMax=0;
    }

   // Correct offset of dataB

    MatrixXd dataA(contLin,4),dataB (contLin,4);// store dataA and dataB time
    for (int i =0; i < contLin; i ++){
        std::cout<<"inside the for i="<<i<<std::endl;

        dataA(i,0)=timeA[i];
        dataA(i,1)=readingA(i,0);
        dataA(i,2)=readingA(i,1);
        dataA(i,3)=readingA(i,2);

        dataB(i,0)=timeB[i];
        dataB(i,1)=readingB(i,0);
        dataB(i,2)=readingB(i,1);
        dataB(i,3)=readingB(i,2);

    }
    std::cout<< "MainWindow::offsetEstimated=== ="<< offsetEstimated <<std::endl;
    for (int i=0; i< dataB.rows(); i++){
        Vector4d myNewVector(((dataB.row(i))(0) + offsetEstimated),(dataB.row(i))(1),(dataB.row(i))(2),(dataB.row(i))(3));
        //contAddedValues++;
        dataB.row(i)<<myNewVector.transpose();
     }

//myInterpolator.performInterpolation(ftype,frequency,dataA,dataB);
//begin before creating performInterpolation
//    myInterpolator.interpolateSerieToFrequency2(frequency, dataA);
//    myInterpolator.interpolateSerieToFrequency2(frequency, dataB);
//    myInterpolator.reduceSequence(10,dataA);
//    std::cout<< "1 lines dataA="<< dataA.rows() <<std::endl;
//    std::cout<< "1 lines dataB="<< dataB.rows() <<std::endl;
//    myInterpolator.interpolate2SeriesFMin(dataB.cols(),dataA,dataB);
//    std::cout<< "2 lines dataA="<< dataA.rows() <<std::endl;
//    std::cout<< "2 lines dataB="<< dataB.rows() <<std::endl;
// end before creating performInterpolation

    //myInterpolator.interpolate2SeriesB(dataA.cols(), dataA, dataB);
   //Check if the time is aligned for A and B

    for (int t=0; t<dataA.rows(); t++){
           std::cout<<"================================================================dataAtime["<<t<<"]="<<dataA(t,0)<<"  dataBtime["<<t<<"]="<<dataB(t,0)<<std::endl;
           if (t>10)
               break;
    }
    int numRows;
    if (dataB.rows() > dataA.rows()){ //after interpolation to fmin it might be possible to adjust number of cols
        numRows= dataA.rows();

    }else numRows=dataB.rows();
    readingB=dataB.block(0,1,numRows,3);
    readingA=dataA.block(0,1,numRows,3);


    // Adapt dataB to Scale if necessary
    //==========================================
    double medScala;//To store the medium value of the estimated scale
    if (myScalaSVD(0) > 1 || myScalaSVD(1) > 1 || myScalaSVD(2) > 1 ){ //if myScaleSVD is not (1,1,1)
        medScala = (myScalaSVD(0)+myScalaSVD(1)+myScalaSVD(2))/3;
        std::cout<< "MainWindow::onEstimateSequence medScala="<< medScala  <<std::endl;
        MatrixXd newB(readingB.rows(),3);
        newB = readingB.block(0,0,readingB.rows(),3);
        std::cout <<"Scale is greater than 1. Divide by Scale dataset B and datasetBB"<<myScalaSVD<<std::endl;
        for (int i= 0; i< newB.rows(); i++){
            VectorXd aRow = newB.row(i);
            //newB.row(i) << aRow(0)/myScalaSVD(0),aRow(1)/myScalaSVD(1),aRow(2)/myScalaSVD(2);
            newB.row(i)<< aRow(0)/medScala,aRow(1)/medScala,aRow(2)/medScala;
         }

        //myRegistrador.rigid_transform_3D(newB,readingA,rotationEstimated,traslationEstimated);ok
        if (way == 0) {//A to B
            myRegistrador.rigid_transform_3D(readingA,newB,rotationEstimated,traslationEstimated);
            std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
            std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
            myRegistrador.applyTransformationsOverData(readingA,dataEstimated,rotationEstimated,traslationEstimated);
            MatrixXd newA(dataEstimated.rows(),3);
            newA = dataEstimated.block(0,0,dataEstimated.rows(),3);

            for (int i= 0; i< newA.rows(); i++){
                VectorXd aRow = newA.row(i);
                newA.row(i)<< aRow(0)*medScala,aRow(1)*medScala,aRow(2)*medScala;
             }
             dataEstimated = newA.block(0,0,newA.rows(),3);
            //myRegistrador.applyTransformationsOverData(newA,dataEstimated,rotationEstimated,traslationEstimated);


        } else { //B to A
            myRegistrador.rigid_transform_3D(newB,readingA,rotationEstimated,traslationEstimated);
            //myRegistrador.rigid_transform_3D(readingA,newB,rotationEstimated,traslationEstimated);
            std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
            std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
            myRegistrador.applyTransformationsOverData(newB,dataEstimated,rotationEstimated,traslationEstimated);
         }

        //myRegistrador.applyTransformationsOverData(newB,dataEstimated,rotationEstimated,traslationEstimated);ok

        /*MatrixXd newA(readingA.rows(),3);
        newA = readingA.block(0,0,readingA.rows(),3);

        for (int i= 0; i< newA.rows(); i++){
            VectorXd aRow = newA.row(i);
            newA.row(i)<< aRow(0)*medScala,aRow(1)*medScala,aRow(2)*medScala;
         }
         */


    } else{
        if (way == 0) { // A to B
        //myRegistrador.rigid_transform_3D(readingB,readingA,rotationEstimated,traslationEstimated);
        myRegistrador.rigid_transform_3D(readingA,readingB,rotationEstimated,traslationEstimated);
        std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
        std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
        //myRegistrador.applyTransformationsOverData(readingB,dataEstimated,rotationEstimated,traslationEstimated);
        myRegistrador.applyTransformationsOverData(readingA,dataEstimated,rotationEstimated,traslationEstimated);

        } else { // B to A

        myRegistrador.rigid_transform_3D(readingB,readingA,rotationEstimated,traslationEstimated);
        std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
        std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
        myRegistrador.applyTransformationsOverData(readingB,dataEstimated,rotationEstimated,traslationEstimated);

        }
    }

    std::cout <<"TRASLATION ESTIMATED----------------------------->"<<traslationEstimated<<std::endl;


    //std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
    ((Winslam*)(myWinSlam))->setEstimatedDataView(dataEstimated);
    dialogShowEstimated =(new DialogShowEstimated (this));
    //dialogScalaTraslaRota->setDataDialog(new DataDialogScalaTraslaRota(1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0));
    double x1 = rotationEstimated.row(0)(0);
    double y1 = rotationEstimated.row(0)(1);
    double z1 = rotationEstimated.row(0)(2);

    double x2 = rotationEstimated.row(1)(0);
    double y2 = rotationEstimated.row(1)(1);
    double z2 = rotationEstimated.row(1)(2);

    double x3 = rotationEstimated.row(2)(0);
    double y3 = rotationEstimated.row(2)(1);
    double z3 = rotationEstimated.row(2)(2);
    Statistics* myStatistics ;
    if (way == 0)
        myStatistics = new Statistics(readingB,dataEstimated);
    else
        myStatistics = new Statistics(readingA,dataEstimated);

    double RMSE = myStatistics->RMSE(myStatistics->getErrorRows());
    std::cout<< "MainWindow:: Statistics----->> R M S E = "<<RMSE <<std::endl;
    std::cout<< "MainWindow:: lines readingB---->> = "<<readingB.rows() <<std::endl;
    std::cout<< "MainWindow:: dataEstimated----->> = "<<dataEstimated.rows() <<std::endl;
    //dataDialogShowEstimated= new DataDialogShowEstimated(myScalaSVD(0),myScalaSVD(1),myScalaSVD(2),traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax);
    //dataDialogShowEstimated= new DataDialogShowEstimated(medScala,medScala,medScala,traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax,1,myGeneratorPCA.getPcaA(),myGeneratorPCA.getPcaB());

    //Calculate Yaw Pitch and Roll from the Rotation Matrix
    Matrix3d myRotationEstimated(3, 3);
    myRotationEstimated = rotationEstimated.block(0,0,3,3);
    Eigen::Matrix< double, 3, 1> ypr=myRotationEstimated.eulerAngles(2,1,0);
    //double y= ypr(0)*180/M_PI;//yaw
    //double p= ypr(1)*180/M_PI;//pitch
    //double r= ypr(2)*180/M_PI;//roll
    double y= ypr(0);//yaw
    double p= ypr(1);//pitch
    double r= ypr(2);//roll
    std::cout<<"rpy====================================================================="<<ypr<<std::endl;

    //dataDialogShowEstimated= new DataDialogShowEstimated(medScala,medScala,medScala,traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax,1,RMSE);

    dataDialogShowEstimated= new DataDialogShowEstimated(medScala,medScala,medScala,traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),y,p,r,timeOffsetEstimated,rMax,way,RMSE);

    dialogShowEstimated->setDataDialog(dataDialogShowEstimated);
    dialogShowEstimated->show();

}
//================================================================================================================
void MainWindow::onEstimateSequenceAtoB(){
    onEstimateSequence(0);
}

//void MainWindow::onEstimateSequenceAtoB(){//Estimate transformations from dataset A to dataset B
//    //frequency=0.005;
//    std::cout<<"onEstimateSequenceAtoB frequency="<<frequency<<std::endl;
//    //myRegistrador.rigid_transform_3D()
//    readingA=readingAbkp.block(0,0,readingAbkp.rows(),3);//to use with Scale and PCA
//    readingB=readingBbkp.block(0,0,readingBbkp.rows(),3);

//    int contLin = readingB.rows();
//    std::cout <<"MainWindow::onEstimateSequence contLin="<<contLin<<std::endl;
//    //Begin Try to estimate Scala using PCA
//    //=========================================
//    MatrixXd AA,BB,AApca,BBpca (contLin,3);
//    MatrixXd pcaA,pcaB;
//    //B = readingB.block(0,0,contLin,3);
//    AA = readingA.block(0,0,readingA.rows(),3);//to use with Scale and PCA
//    BB = readingB.block(0,0,readingB.rows(),3);//to use with Scale and PCA

//    myGeneratorPCA.calculatePCAbySVD(0,AA, AApca, pcaA);//A is converted to PCA. Important,inside this function also is calculated A.rowwise() - A.colwise().mean(). A is converted to a newA
//    myGeneratorPCA.calculatePCAbySVD(0,BB, BBpca, pcaB);//B is converted to PCA. Important,inside this function also is calculated B.rowwise() - B.colwise().mean(). B is converted to a newB
//    myGeneratorPCA.setPcaA(pcaA);
//    myGeneratorPCA.setPcaB(pcaB);

//    Vector3d myScalaSVD = myFindScala.getScalaSVD(AApca,BBpca);
//    std::cout <<"MainWindow::onEstimateSequence myScalaSVD="<<myScalaSVD<<std::endl;

//    // End Try to estimate Scala using PCA
//    //==========================================

//    // Try to find Time offset
//    //==========================================
//    double offsetEstimated = 0;
//    if (timeOffset > 0) {
//            int maxLine = readingA.rows();
//            int interval = 10;
//            double anOffset = timeOffset;
//            double correlationValue = 0;
//            //double correlationValue = myFindOffset.calculateOffsetXYZ( maxLine,interval, anOffset,offsetEstimated, AApca,  BBpca);//ok
//            //timeOffsetEstimated= offsetEstimated;
//            //std::cout<<"timeOffsetEstimated="<<timeOffsetEstimated<<std::endl;

//        // Calculate interpolation with AApca and BBpca
//        // But AApca and BBpca has only 3 coordinates, needs to add the 4th coordinate , which is TIME

//        //anOffset=0;
//        MatrixXd tAApca(contLin,4),tBBpca (contLin,4);// store aapca and bbpca with time
//        for (int i =0; i < contLin; i ++){
//            std::cout<<"inside the for i="<<i<<std::endl;
//            //tAApca(i,0)=AA(i,0);
//            tAApca(i,0)=timeA[i];
//            tAApca(i,1)=AApca(i,0);
//            tAApca(i,2)=AApca(i,1);
//            tAApca(i,3)=AApca(i,2);
//            //tBBpca(i,0)=BB(i,0);
//            tBBpca(i,0)=timeB[i];
//            tBBpca(i,1)=BBpca(i,0);
//            tBBpca(i,2)=BBpca(i,1);
//            tBBpca(i,3)=BBpca(i,2);

//        }
//        //rMax=myInterpolator.calculateOffsetWithInterpolation(maxLine,interval, anOffset,offsetEstimated, tAApca,  tBBpca);
//        rMax=myInterpolator.calculateOffsetWithInterpolation(maxLine,interval,offsetEstimated, tAApca,  tBBpca);
//        timeOffsetEstimated= offsetEstimated;
//        std::cout<<"timeOffsetEstimated2="<<timeOffsetEstimated<<std::endl;
//        std::cout<<"timeOffset EXPECTED="<<timeOffset<<std::endl;

//        //Apply estimated offset over B
//        for (int i=0; i< tBBpca.rows(); i++){
//            Vector4d myNewVector(((tBBpca.row(i))(0) + offsetEstimated),(tBBpca.row(i))(1),(tBBpca.row(i))(2),(tBBpca.row(i))(3));
//            //contAddedValues++;



//            tBBpca.row(i)<<myNewVector.transpose();
//         }
//         myInterpolator.interpolateSerieToFrequency2(frequency, tAApca);
//        //Check if the time is aligned for A and B
//         for (int t=0; t<100; t++){
//                std::cout<<"================================================================timeA["<<t<<"]="<<tAApca(t,0)<<"  timeB["<<t<<"]="<<tBBpca(t,0)<<std::endl;
//         }

//    } else {
//         timeOffsetEstimated=0;
//         rMax=0;
//    }

//   // Correct offset of dataB

//    MatrixXd dataA(contLin,4),dataB (contLin,4);// store dataA and dataB time
//    for (int i =0; i < contLin; i ++){
//        std::cout<<"inside the for i="<<i<<std::endl;

//        dataA(i,0)=timeA[i];
//        dataA(i,1)=readingA(i,0);
//        dataA(i,2)=readingA(i,1);
//        dataA(i,3)=readingA(i,2);

//        dataB(i,0)=timeB[i];
//        dataB(i,1)=readingB(i,0);
//        dataB(i,2)=readingB(i,1);
//        dataB(i,3)=readingB(i,2);

//    }
//    std::cout<< "MainWindow::offsetEstimated=== ="<< offsetEstimated <<std::endl;
//    for (int i=0; i< dataB.rows(); i++){
//        Vector4d myNewVector(((dataB.row(i))(0) + offsetEstimated),(dataB.row(i))(1),(dataB.row(i))(2),(dataB.row(i))(3));
//        //contAddedValues++;
//        dataB.row(i)<<myNewVector.transpose();
//     }
//    myInterpolator.interpolateSerieToFrequency2(frequency, dataA);
//    myInterpolator.interpolateSerieToFrequency2(frequency, dataB);
//    myInterpolator.reduceSequence(10,dataA);
//    std::cout<< "1 lines dataA="<< dataA.rows() <<std::endl;
//    std::cout<< "1 lines dataB="<< dataB.rows() <<std::endl;
//    myInterpolator.interpolate2SeriesFMin(dataB.cols(),dataA,dataB);
//    std::cout<< "2 lines dataA="<< dataA.rows() <<std::endl;
//    std::cout<< "2 lines dataB="<< dataB.rows() <<std::endl;

//    //myInterpolator.interpolate2SeriesB(dataA.cols(), dataA, dataB);
//   //Check if the time is aligned for A and B
//    for (int t=0; t<100; t++){
//           std::cout<<"================================================================dataAtime["<<t<<"]="<<dataA(t,0)<<"  dataBtime["<<t<<"]="<<dataB(t,0)<<std::endl;
//    }
//    int numRows;
//    if (dataB.rows() > dataA.rows()){ //after interpolation to fmin it might be possible to adjust number of cols
//        numRows= dataA.rows();

//    }else numRows=dataB.rows();
//    readingB=dataB.block(0,1,numRows,3);
//    readingA=dataA.block(0,1,numRows,3);


//    // Adapt dataB to Scale if necessary
//    //==========================================
//    double medScala;//To store the medium value of the estimated scale
//    if (myScalaSVD(0) > 1 || myScalaSVD(1) > 1 || myScalaSVD(2) > 1 ){ //if myScaleSVD is not (1,1,1)
//        medScala = (myScalaSVD(0)+myScalaSVD(1)+myScalaSVD(2))/3;
//        std::cout<< "MainWindow::onEstimateSequence medScala="<< medScala  <<std::endl;
//        MatrixXd newB(readingB.rows(),3);
//        newB = readingB.block(0,0,readingB.rows(),3);
//        std::cout <<"Scale is greater than 1. Divide by Scale dataset B and datasetBB"<<myScalaSVD<<std::endl;
//        for (int i= 0; i< newB.rows(); i++){
//            VectorXd aRow = newB.row(i);
//            //newB.row(i) << aRow(0)/myScalaSVD(0),aRow(1)/myScalaSVD(1),aRow(2)/myScalaSVD(2);
//            newB.row(i)<< aRow(0)/medScala,aRow(1)/medScala,aRow(2)/medScala;
//         }

//        //myRegistrador.rigid_transform_3D(newB,readingA,rotationEstimated,traslationEstimated);ok
//        myRegistrador.rigid_transform_3D(readingA,newB,rotationEstimated,traslationEstimated);
//        std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
//        std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
//        //myRegistrador.applyTransformationsOverData(newB,dataEstimated,rotationEstimated,traslationEstimated);ok

//        /*MatrixXd newA(readingA.rows(),3);
//        newA = readingA.block(0,0,readingA.rows(),3);

//        for (int i= 0; i< newA.rows(); i++){
//            VectorXd aRow = newA.row(i);
//            newA.row(i)<< aRow(0)*medScala,aRow(1)*medScala,aRow(2)*medScala;
//         }
//         */

//        myRegistrador.applyTransformationsOverData(readingA,dataEstimated,rotationEstimated,traslationEstimated);
//        MatrixXd newA(dataEstimated.rows(),3);
//        newA = dataEstimated.block(0,0,dataEstimated.rows(),3);

//        for (int i= 0; i< newA.rows(); i++){
//            VectorXd aRow = newA.row(i);
//            newA.row(i)<< aRow(0)*medScala,aRow(1)*medScala,aRow(2)*medScala;
//         }
//         dataEstimated = newA.block(0,0,newA.rows(),3);
//        //myRegistrador.applyTransformationsOverData(newA,dataEstimated,rotationEstimated,traslationEstimated);


//    } else{

//        //myRegistrador.rigid_transform_3D(readingB,readingA,rotationEstimated,traslationEstimated);
//        myRegistrador.rigid_transform_3D(readingA,readingB,rotationEstimated,traslationEstimated);
//        std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
//        std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
//        //myRegistrador.applyTransformationsOverData(readingB,dataEstimated,rotationEstimated,traslationEstimated);
//        myRegistrador.applyTransformationsOverData(readingA,dataEstimated,rotationEstimated,traslationEstimated);
//    }

//    std::cout <<"TRASLATION ESTIMATED----------------------------->"<<traslationEstimated<<std::endl;


//    //std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
//    ((Winslam*)(myWinSlam))->setEstimatedDataView(dataEstimated);
//    dialogShowEstimated =(new DialogShowEstimated (this));
//    //dialogScalaTraslaRota->setDataDialog(new DataDialogScalaTraslaRota(1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0));
//    double x1 = rotationEstimated.row(0)(0);
//    double y1 = rotationEstimated.row(0)(1);
//    double z1 = rotationEstimated.row(0)(2);

//    double x2 = rotationEstimated.row(1)(0);
//    double y2 = rotationEstimated.row(1)(1);
//    double z2 = rotationEstimated.row(1)(2);

//    double x3 = rotationEstimated.row(2)(0);
//    double y3 = rotationEstimated.row(2)(1);
//    double z3 = rotationEstimated.row(2)(2);
//    Statistics* myStatistics ;
//    myStatistics = new Statistics(readingB,dataEstimated);
//    double RMSE = myStatistics->RMSE(myStatistics->getErrorRows());
//    std::cout<< "MainWindow:: Statistics----->> R M S E = "<<RMSE <<std::endl;
//    std::cout<< "MainWindow:: lines readingB---->> = "<<readingB.rows() <<std::endl;
//    std::cout<< "MainWindow:: dataEstimated----->> = "<<dataEstimated.rows() <<std::endl;
//    //dataDialogShowEstimated= new DataDialogShowEstimated(myScalaSVD(0),myScalaSVD(1),myScalaSVD(2),traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax);
//    //dataDialogShowEstimated= new DataDialogShowEstimated(medScala,medScala,medScala,traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax,1,myGeneratorPCA.getPcaA(),myGeneratorPCA.getPcaB());

//    //Calculate Yaw Pitch and Roll from the Rotation Matrix
//    Matrix3d myRotationEstimated(3, 3);
//    myRotationEstimated = rotationEstimated.block(0,0,3,3);
//    Eigen::Matrix< double, 3, 1> ypr=myRotationEstimated.eulerAngles(2,1,0);
//    double y= ypr(0)*180/M_PI;//yaw
//    double p= ypr(1)*180/M_PI;//pitch
//    double r= ypr(2)*180/M_PI;//roll
//    std::cout<<"rpy====================================================================="<<ypr<<std::endl;

//    //dataDialogShowEstimated= new DataDialogShowEstimated(medScala,medScala,medScala,traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax,1,RMSE);
//    dataDialogShowEstimated= new DataDialogShowEstimated(medScala,medScala,medScala,traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),y,p,r,timeOffsetEstimated,rMax,1,RMSE);

//    dialogShowEstimated->setDataDialog(dataDialogShowEstimated);
//    dialogShowEstimated->show();

//}
//============================================================================================================
void MainWindow::onEstimateSequenceBtoA(){//estimate transformations from dataset B to dataset A
    onEstimateSequence(1);
}
//void MainWindow::onEstimateSequenceBtoA(){//estimate transformations from dataset B to dataset A
//    //frequency=0.005;
//    std::cout<<"onEstimateSequenceBtoA frequency="<<frequency<<std::endl;
//    //myRegistrador.rigid_transform_3D()
//    readingA=readingAbkp.block(0,0,readingAbkp.rows(),3);//to use with Scale and PCA
//    readingB=readingBbkp.block(0,0,readingBbkp.rows(),3);
//    int contLin = readingB.rows();
//    std::cout <<"MainWindow::onEstimateSequence contLin="<<contLin<<std::endl;
//    //Begin Try to estimate Scala using PCA
//    //=========================================
//    MatrixXd AA,BB,AApca,BBpca (contLin,3);
//    MatrixXd pcaA,pcaB;
//    //B = readingB.block(0,0,contLin,3);
//    AA = readingA.block(0,0,readingA.rows(),3);//to use with Scale and PCA
//    BB = readingB.block(0,0,readingB.rows(),3);//to use with Scale and PCA

//    myGeneratorPCA.calculatePCAbySVD(0,AA, AApca, pcaA);//A is converted to PCA. Important,inside this function also is calculated A.rowwise() - A.colwise().mean(). A is converted to a newA
//    std::cout <<"MainWindow::pcaA)))))))))))))))))))))))))))))))))))))))))))))))))))))))))="<<pcaA<<std::endl;
//    myGeneratorPCA.calculatePCAbySVD(0,BB, BBpca, pcaB);//B is converted to PCA. Important,inside this function also is calculated B.rowwise() - B.colwise().mean(). B is converted to a newB
//    std::cout <<"MainWindow::pcaB)))))))))))))))))))))))))))))))))))))))))))))))))))))))))="<<pcaB<<std::endl;
//    myGeneratorPCA.setPcaA(pcaA);
//    myGeneratorPCA.setPcaB(pcaB);


//    Vector3d myScalaSVD = myFindScala.getScalaSVD(AApca,BBpca);
//    std::cout <<"MainWindow::onEstimateSequence myScalaSVD="<<myScalaSVD<<std::endl;

//    // End Try to estimate Scala using PCA
//    //==========================================

//    // Try to find Time offset
//    //==========================================
//    double offsetEstimated = 0;
//    if (timeOffset > 0) {
//            int maxLine = readingA.rows();
//            int interval = 10;
//            double anOffset = timeOffset;
//            double correlationValue = 0;
//            //double correlationValue = myFindOffset.calculateOffsetXYZ( maxLine,interval, anOffset,offsetEstimated, AApca,  BBpca);//ok
//            //timeOffsetEstimated= offsetEstimated;
//            //std::cout<<"timeOffsetEstimated="<<timeOffsetEstimated<<std::endl;

//        // Calculate interpolation with AApca and BBpca
//        // But AApca and BBpca has only 3 coordinates, needs to add the 4th coordinate , which is TIME

//        //anOffset=0;
//        MatrixXd tAApca(contLin,4),tBBpca (contLin,4);// store aapca and bbpca with time
//        for (int i =0; i < contLin; i ++){
//            std::cout<<"inside the for i="<<i<<std::endl;
//            //tAApca(i,0)=AA(i,0);
//            tAApca(i,0)=timeA[i];
//            tAApca(i,1)=AApca(i,0);
//            tAApca(i,2)=AApca(i,1);
//            tAApca(i,3)=AApca(i,2);
//            //tBBpca(i,0)=BB(i,0);
//            tBBpca(i,0)=timeB[i];
//            tBBpca(i,1)=BBpca(i,0);
//            tBBpca(i,2)=BBpca(i,1);
//            tBBpca(i,3)=BBpca(i,2);

//        }
//        //rMax=myInterpolator.calculateOffsetWithInterpolation(maxLine,interval, anOffset,offsetEstimated, tAApca,  tBBpca);
//        rMax=myInterpolator.calculateOffsetWithInterpolation(maxLine,interval,offsetEstimated, tAApca,  tBBpca);
//        timeOffsetEstimated= offsetEstimated;
//        std::cout<<"timeOffsetEstimated2="<<timeOffsetEstimated<<std::endl;
//        std::cout<<"timeOffset EXPECTED="<<timeOffset<<std::endl;

//        //Apply estimated offset over A
//        for (int i=0; i< tBBpca.rows(); i++){
//            Vector4d myNewVector(((tBBpca.row(i))(0) + offsetEstimated),(tBBpca.row(i))(1),(tBBpca.row(i))(2),(tBBpca.row(i))(3));
//            //contAddedValues++;



//            tBBpca.row(i)<<myNewVector.transpose();
//         }
//        myInterpolator.interpolateSerieToFrequency2(frequency, tAApca);
//       //Check if the time is aligned for A and B
//        for (int t=0; t<100; t++){
//               std::cout<<"================================================================timeA["<<t<<"]="<<tAApca(t,0)<<"  timeB["<<t<<"]="<<tBBpca(t,0)<<std::endl;
//        }

//   } else {
//        timeOffsetEstimated=0;
//        rMax=0;
//   }


//    // Correct offset of dataB

//     MatrixXd dataA(contLin,4),dataB (contLin,4);// store dataA and dataB time
//     for (int i =0; i < contLin; i ++){
//         std::cout<<"inside the for i="<<i<<std::endl;

//         dataA(i,0)=timeA[i];
//         dataA(i,1)=readingA(i,0);
//         dataA(i,2)=readingA(i,1);
//         dataA(i,3)=readingA(i,2);

//         dataB(i,0)=timeB[i];
//         dataB(i,1)=readingB(i,0);
//         dataB(i,2)=readingB(i,1);
//         dataB(i,3)=readingB(i,2);

//     }
//     std::cout<< "MainWindow::offsetEstimated=== ="<< offsetEstimated <<std::endl;
//     for (int i=0; i< dataB.rows(); i++){
//         Vector4d myNewVector(((dataB.row(i))(0) + offsetEstimated),(dataB.row(i))(1),(dataB.row(i))(2),(dataB.row(i))(3));
//         //contAddedValues++;
//         dataB.row(i)<<myNewVector.transpose();
//      }

//     myInterpolator.interpolateSerieToFrequency2(frequency, dataA);
//     myInterpolator.interpolateSerieToFrequency2(frequency, dataB);
//     myInterpolator.reduceSequence(10,dataA);
//     std::cout<< "1 lines dataA="<< dataA.rows() <<std::endl;
//     std::cout<< "1 lines dataB="<< dataB.rows() <<std::endl;
//     myInterpolator.interpolate2SeriesFMin(dataB.cols(),dataA,dataB);
//     std::cout<< "2 lines dataA="<< dataA.rows() <<std::endl;
//     std::cout<< "2 lines dataB="<< dataB.rows() <<std::endl;
//     //myInterpolator.interpolate2SeriesB(dataA.cols(), dataA, dataB);
//    //Check if the time is aligned for A and B
//     for (int t=0; t<100; t++){
//            std::cout<<"================================================================dataAtime["<<t<<"]="<<dataA(t,0)<<"  dataBtime["<<t<<"]="<<dataB(t,0)<<std::endl;
//     }
//     int numRows;
//     if (dataB.rows() > dataA.rows()){ //after interpolation to fmin it might be possible to adjust number of cols
//         numRows= dataA.rows();

//     }else numRows=dataB.rows();
//     readingB=dataB.block(0,1,numRows,3);
//     readingA=dataA.block(0,1,numRows,3);



//    // Adapt dataB to Scale if necessary
//    //==========================================
//    double medScala;//To store the medium value of the estimated scale
//    if (myScalaSVD(0) > 1 || myScalaSVD(1) > 1 || myScalaSVD(2) > 1 ){ //if myScaleSVD is not (1,1,1)
//        medScala = (myScalaSVD(0)+myScalaSVD(1)+myScalaSVD(2))/3;
//        std::cout<< "MainWindow::onEstimateSequence medScala="<< medScala  <<std::endl;
//        MatrixXd newB(readingB.rows(),3);
//        newB = readingB.block(0,0,readingB.rows(),3);
//        std::cout <<"Scale is greater than 1. Divide by Scale dataset B and datasetBB"<<myScalaSVD<<std::endl;
//        for (int i= 0; i< newB.rows(); i++){
//            VectorXd aRow = newB.row(i);
//            //newB.row(i) << aRow(0)/myScalaSVD(0),aRow(1)/myScalaSVD(1),aRow(2)/myScalaSVD(2);
//            newB.row(i)<< aRow(0)/medScala,aRow(1)/medScala,aRow(2)/medScala;
//         }

//        myRegistrador.rigid_transform_3D(newB,readingA,rotationEstimated,traslationEstimated);
//        //myRegistrador.rigid_transform_3D(readingA,newB,rotationEstimated,traslationEstimated);
//        std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
//        std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
//        myRegistrador.applyTransformationsOverData(newB,dataEstimated,rotationEstimated,traslationEstimated);


//        //myRegistrador.applyTransformationsOverData(readingA,dataEstimated,rotationEstimated,traslationEstimated);
//        /*
//        MatrixXd newA(dataEstimated.rows(),3);
//        newA = dataEstimated.block(0,0,dataEstimated.rows(),3);

//        for (int i= 0; i< newA.rows(); i++){
//            VectorXd aRow = newA.row(i);
//            newA.row(i)<< aRow(0)*medScala,aRow(1)*medScala,aRow(2)*medScala;
//         }
//         dataEstimated = newA.block(0,0,newA.rows(),3);
//        */
//        //myRegistrador.applyTransformationsOverData(newA,dataEstimated,rotationEstimated,traslationEstimated);


//    } else{

//        myRegistrador.rigid_transform_3D(readingB,readingA,rotationEstimated,traslationEstimated);
//        std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
//        std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
//        myRegistrador.applyTransformationsOverData(readingB,dataEstimated,rotationEstimated,traslationEstimated);
//    }

//    std::cout <<"TRASLATION ESTIMATED----------------------------->"<<traslationEstimated<<std::endl;


//    //std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
//    ((Winslam*)(myWinSlam))->setEstimatedDataView(dataEstimated);
//    dialogShowEstimated =(new DialogShowEstimated (this));
//    //dialogScalaTraslaRota->setDataDialog(new DataDialogScalaTraslaRota(1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0));
//    double x1 = rotationEstimated.row(0)(0);
//    double y1 = rotationEstimated.row(0)(1);
//    double z1 = rotationEstimated.row(0)(2);

//    double x2 = rotationEstimated.row(1)(0);
//    double y2 = rotationEstimated.row(1)(1);
//    double z2 = rotationEstimated.row(1)(2);

//    double x3 = rotationEstimated.row(2)(0);
//    double y3 = rotationEstimated.row(2)(1);
//    double z3 = rotationEstimated.row(2)(2);
//    Statistics* myStatistics ;
//    myStatistics = new Statistics(readingA,dataEstimated);
//    double RMSE = myStatistics->RMSE(myStatistics->getErrorRows());
//    std::cout<< "MainWindow:: Statistics----->> R M S E = "<<RMSE <<std::endl;
//    std::cout<< "MainWindow:: lines readingA---->> = "<<readingA.rows() <<std::endl;
//    std::cout<< "MainWindow:: dataEstimated----->> = "<<dataEstimated.rows() <<std::endl;
//    //dataDialogShowEstimated= new DataDialogShowEstimated(myScalaSVD(0),myScalaSVD(1),myScalaSVD(2),traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax);
//    //dataDialogShowEstimated= new DataDialogShowEstimated(medScala,medScala,medScala,traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax,1,myGeneratorPCA.getPcaA(),myGeneratorPCA.getPcaB());
//    //dataDialogShowEstimated= new DataDialogShowEstimated(myScalaSVD(0),myScalaSVD(1),myScalaSVD(2),traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),x1,y1,z1,x2,y2,z2,x3,y3,z3,timeOffsetEstimated,rMax);
//    //Calculate Yaw Pitch and Roll from the Rotation Matrix
//    Matrix3d myRotationEstimated(3, 3);
//    myRotationEstimated = rotationEstimated.block(0,0,3,3);
//    Eigen::Matrix< double, 3, 1> ypr=myRotationEstimated.eulerAngles(2,1,0);
//    double y= ypr(0)*180/M_PI;//yaw must be converted from radian to grades
//    double p= ypr(1)*180/M_PI;//pitch must be converted from radian to grades
//    double r= ypr(2)*180/M_PI;//roll must be converted from radian to grades
//    std::cout<<"rpy====================================================================="<<ypr<<std::endl;

//    dataDialogShowEstimated= new DataDialogShowEstimated(medScala,medScala,medScala,traslationEstimated(0),traslationEstimated(1),traslationEstimated(2),y,p,r,timeOffsetEstimated,rMax,2,RMSE);

//    dialogShowEstimated->setDataDialog(dataDialogShowEstimated);
//    dialogShowEstimated->show();



//}
//============================================================================================================
void MainWindow::loadFile(const QString &fileName)
{
    QFile file(fileName);
    int maxLine = 3000;
    //Eigen::MatrixXd readingA (maxLine,3);
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

        MatrixXd newA(maxLine,3);
        while ( (infileA >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
                    std::cout <<"contLin="<<contLin<<std::endl;
                    newA.row(contLin)<< rx,ry,rz;
                    timeA[contLin]= timestamp;
                    std::cout <<"contLin="<<contLin<<std::endl;
                    contLin ++;


        }


        readingA=newA.block(0,0,contLin,3);
        readingAbkp=newA.block(0,0,contLin,3);
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
//============================================================================================================
void MainWindow::setScala(double X, double Y, double Z){
     std::cout<< "MainWindow.setScala" <<std::endl;
    ((Winslam*)(myWinSlam))->setScala(X,Y,Z);
}
//============================================================================================================
void MainWindow::setTrasla(double X, double Y, double Z){
    ((Winslam*)(myWinSlam))->setTrasla(X,Y,Z);
    //performModifySequence(X,Y,Z, X,Y, Z,0,0,0,0,0);
}
//============================================================================================================
void MainWindow::performModifySequence(double scalaX,double scalaY,double scalaZ, double traslaX,double traslaY, double traslaZ,double rotaX,double rotaY,double rotaZ,double gNoise, double cNoise, double offset,int fType,double freq){
     dataDialogScalaTraslaRota->setScaleX(scalaX);
     dataDialogScalaTraslaRota->setScaleY(scalaY);
     dataDialogScalaTraslaRota->setScaleZ(scalaZ);
     dataDialogScalaTraslaRota->setTraslaX(traslaX);
     dataDialogScalaTraslaRota->setTraslaY(traslaY);
     dataDialogScalaTraslaRota->setTraslaZ(traslaZ);
     dataDialogScalaTraslaRota->setRotaX(rotaX);
     dataDialogScalaTraslaRota->setRotaY(rotaY);
     dataDialogScalaTraslaRota->setRotaZ(rotaZ);
     dataDialogScalaTraslaRota->setGaussianNoiseDeviation(gNoise);
     dataDialogScalaTraslaRota->setGaussianNoiseDeviation(cNoise);
     dataDialogScalaTraslaRota->setTimeOffset(offset);
     dataDialogScalaTraslaRota->setFrequencyType(fType);
     //dataDialogScalaTraslaRota->setPcaIndex(pcaIndex);
     dataDialogScalaTraslaRota->setFrequency(freq);


     int maxLine = 3000;
     Point3D  myScala;
     myScala.setXYZ(scalaX,scalaY,scalaZ);

     Point3D miTraslacion ;
     miTraslacion.setXYZ(traslaX,traslaY,traslaZ);

     myOutputFileName="miSalidaContaminadaQT.txt";
     std::cout <<"mainwindow.performModifySequence  gNoise="<<gNoise<< " cNoise="<<cNoise<<std::endl;
     /*
     char axisToRotate='X';
     double rotationGrades = 0;

     if (rotaX>0) {
         rotationGrades = rotaX;
         axisToRotate = 'X';

     }else if (rotaY>0){
         rotationGrades = rotaY;
         axisToRotate = 'Y';
     }else if (rotaZ>0){
         rotationGrades = rotaZ;
         axisToRotate = 'Z';
     }else {
         rotationGrades = 0;
     }
     */


     if (gNoise > 0.0 & cNoise >0.0) {
         //myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,rotationGrades,'X',1,1,offset,fType,freq);
         myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,rotaX,rotaY,rotaZ,1,1,offset,fType,freq);
     } else if (gNoise > 0.0 & cNoise <=0.0) {
        //myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,rotationGrades,'X',1,0,offset,fType,freq);
        myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,rotaX,rotaY,rotaZ,1,0,offset,fType,freq);
     } else if (gNoise <= 0 & cNoise >0.0){
         //myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,rotationGrades,'X',0,1,offset,fType,freq);
         myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,rotaX,rotaY,rotaZ,0,1,offset,fType,freq);
     } else  //myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,rotationGrades,'X',0,0,offset,fType,freq);
         myTransformador.createContaminatedSequence(myInputFileName,myOutputFileName,miTraslacion,myScala,rotaX,rotaY,rotaZ,0,0,offset,fType,freq);

     // Reading input file B, with new dataset contaminated
     std::ifstream infileB( "miSalidaContaminadaQT.txt" );
     //Eigen::MatrixXd readingB (maxLine,3);
     double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
     int contLin=0;

     MatrixXd newB(maxLine,3);
     while ( (infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
                 //std::cout <<"contLin="<<contLin<<std::endl;
                 newB.row(contLin)<< rx,ry,rz;
                 timeB[contLin]= timestamp;
                 std::cout <<"contLin="<<contLin<<std::endl;
                 contLin ++;


     }


     readingB=newB.block(0,0,contLin,3);
     readingBbkp=newB.block(0,0,contLin,3);
     infileB.close();
     ((Winslam*)(myWinSlam))->setContaminatedDataView(readingB);
     timeOffset=offset;
     frequency=freq;
     ftype=fType;

}


