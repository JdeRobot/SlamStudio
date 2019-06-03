
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
#include <QProgressBar>
#include "Eigen/Dense"
#include "transformador2/Transformador.h"
#include "Statistics/Statistics.h"
#include "Properties/ReaderProperties.h"
#include "configuration.h"
//#include "dialogmessage.h"
#include <QThread>
//#include "transformador2/Point3D.h"
//#include <QGLWidget>


MainWindow::MainWindow()
{
    Configuration myConfiguration;
    Configuration::setMaxLines(50000);
    Configuration::setStepOffset(1/100.0);
    Configuration::setWindowOffset(3);
    int maxLine = 3000;
    Eigen::MatrixXd newMatrixA (maxLine,3);
    readingA = newMatrixA;
    Eigen::MatrixXd newMatrixB (maxLine,3);
    readingB = newMatrixB;
    Eigen::MatrixXd newMatrixEstimated (maxLine,3);
    dataEstimated = newMatrixEstimated;
    QMenuBar *menuBar = new QMenuBar;
    QMenu *menuWindow = menuBar->addMenu(tr("&File"));
    QAction *openFileA = new QAction(menuWindow);
    openFileA->setText(tr("Open dataset A"));
    menuWindow->addAction(openFileA);
    connect(openFileA, &QAction::triggered, this, &MainWindow::onOpenFileA);
    QAction *openFileB = new QAction(menuWindow);
    openFileB->setText(tr("Open dataset B"));
    menuWindow->addAction(openFileB);
    connect(openFileB, &QAction::triggered, this, &MainWindow::onOpenFileB);
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


    QAction *estimateSequenceAtoB_RANSAC = new QAction(menuWindow);
    estimateSequenceAtoB_RANSAC->setText(tr("Estimate sequence A to B with RANSAC"));
    menuEstimate->addAction(estimateSequenceAtoB_RANSAC);
    connect(estimateSequenceAtoB_RANSAC, &QAction::triggered, this, &MainWindow::onEstimateSequenceAtoB_RANSAC);

    QAction *estimateSequenceBtoA_RANSAC = new QAction(menuWindow);
    estimateSequenceBtoA_RANSAC->setText(tr("Estimate sequence B to A with RANSAC"));
    menuEstimate->addAction(estimateSequenceBtoA_RANSAC);
    connect(estimateSequenceBtoA_RANSAC, &QAction::triggered, this, &MainWindow::onEstimateSequenceBtoA_RANSAC);



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

    QMenu *menuConf = menuBar->addMenu(tr("&Configuration"));
    QAction *setConfParameters = new QAction(menuWindow);
    setConfParameters->setText(tr("modify Parameters"));
    menuConf->addAction(setConfParameters);
    connect(setConfParameters,&QAction::triggered,this, &MainWindow::onModifyParameters);

    statusLabel=new QLabel("");
    statusProgressBar = new QProgressBar(this);
    setMenuBar(menuBar);
    statusBar()->addWidget(statusLabel,1);

    //statusBar()->showMessage(tr("Status Bar"),2000);
    statusBar()->showMessage(tr("Status Bar"));
    statusBar()->addPermanentWidget(statusProgressBar,0);
    //statusBar()->addWidget(statusProgressBar,1);
    statusProgressBar->setValue(100);
    statusProgressBar->setValue(0);

    //setProperties();
    //setMenuBar(menuBar);

    if (!centralWidget())
        //setCentralWidget(new Window(this));
        myWinSlam=(QWidget*)(new Winslam(this));
        setCentralWidget(myWinSlam);
    //onAddNew();
    dataDialogScalaTraslaRota= new DataDialogScalaTraslaRota(1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0.005);
    //dialogMessage = new DialogMessage(this);
    dataDialogParameters = new DataDialogParameters(50000,0.01,3);
    std::cout << "end" << std::endl ;
}

void MainWindow::setProperties(){
//    ReaderProperties myReaderProperties = ReaderProperties("/tmp/myProperties.txt");
//    //myReaderProperties.readFile("/tmp/myProperties.txt");
//    string aString = myReaderProperties.getPropertyValue("interpolationStep");
//    interpolationStep=stod(myReaderProperties.getPropertyValue("interpolationStep"));
//    maxLineInputFile=stod(myReaderProperties.getPropertyValue("maxLineInputFile"));
//    maxLineInterpolation=stod(myReaderProperties.getPropertyValue("maxLineInterpolation"));
//    offsetLimit=stod(myReaderProperties.getPropertyValue("offsetLimit"));
    //maxLineInputFile=5000;
    //maxLineInterpolation=50000;
    //interpolationStep=0.001;
    //offsetLimit=5.0;
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

void MainWindow::onOpenFileA()
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
    if (!fileName.isEmpty()) {
        MatrixXd newData;
        loadFile(fileName,newData,0);
        readingA=newData.block(0,0,newData.rows(),7);
        readingAbkp=readingA.block(0,0,readingA.rows(),7);
        ((Winslam*)(myWinSlam))->setDataView(readingA);

    }

}

void MainWindow::onOpenFileB()
{
    /*if (!centralWidget())
        setCentralWidget(new Window(this));
    else
        QMessageBox::information(0, tr("Cannot add new window"), tr("Already occupied. Undock first."));
    */
    //infoLabel->setText(tr("Invoked <b>File|Open</b>"));
    QString fileName = QFileDialog::getOpenFileName(this);
    std::string fname = fileName.toStdString();
    /*char * cstr = new char [fname.size()+1];
    strcpy( cstr, fname.c_str() );
    myInputFileName= cstr;
    if (!fileName.isEmpty()) {
        MatrixXd newData;
        loadFile(fileName,newData,1);
        readingB=newData.block(0,0,newData.rows(),7);
        readingBbkp=readingB.block(0,0,readingB.rows(),7);
        ((Winslam*)(myWinSlam))->setContaminatedDataView(readingB);

    }
    */

    std::ifstream infileB( fname );
    //Eigen::MatrixXd readingB (maxLine,3);
    double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
    int contLin=0, maxLine=10000;

    MatrixXd newB(maxLine,7);
    while ( (infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
                //std::cout <<"contLin="<<contLin<<std::endl;
                if (contLin >= maxLine) {
                    std::cout <<"ERROR el numero de lineas excede el máximo permitido="<<contLin<<std::endl;
                    break;
                }
                newB.row(contLin)<< rx,ry,rz,q1,q2,q3,q4;
                timeB[contLin]= timestamp;
                std::cout <<"contLin="<<contLin<<std::endl;
                contLin ++;


    }


    readingB=newB.block(0,0,contLin,7);
    readingBbkp=newB.block(0,0,contLin,7);
    infileB.close();
    ((Winslam*)(myWinSlam))->setContaminatedDataView(readingB);

    //to indicate that frequency must be calculated for datasetB
    dataSetB_isLoaded_NotTransformed=true;

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
    //dialogMessage->show();
    std::cout<<"onModifySequence.dataDialogScalaTraslaRota->getCosmicNoiseDeviation()"<<dataDialogScalaTraslaRota->getCosmicNoiseDeviation();
    std::cout<< "FIN onModifySequence" <<std::endl;
}
//================================================================================================================

void MainWindow::onEstimateSequence(int way,bool RANSAC){//Estimate transformations from dataset A to dataset B or dataset B to dataset A
    //frequency=0.005;
    float rMax=0.0;


    std::cout<<"onEstimateSequenceAtoB frequency="<<frequency<<std::endl;
    std::cout<<"onEstimateSequenceAtoB frequency="<<ftype<<std::endl;
    //myRegistrador.rigid_transform_3D()
    readingA=readingAbkp.block(0,0,readingAbkp.rows(),3);//to use with Scale and PCA
    readingB=readingBbkp.block(0,0,readingBbkp.rows(),3);
    int contLinA = readingA.rows();
    int contLinB = readingB.rows();
    std::cout <<"MainWindow::onEstimateSequence contLin="<<contLinA<<std::endl;
    //Begin Try to estimate Scala using PCA
    //=========================================
    MatrixXd AA(contLinA,3),BB(contLinB,3),AApca(contLinA,3),BBpca (contLinB,3);
    MatrixXd pcaA,pcaB;
    //B = readingB.block(0,0,contLin,3);
    AA = readingA.block(0,0,readingA.rows(),3);//to use with Scale and PCA
    BB = readingB.block(0,0,readingB.rows(),3);//to use with Scale and PCA

    myGeneratorPCA.calculatePCAbySVD(0,AA, AApca, pcaA);//A is converted to PCA. Important,inside this function also is calculated A.rowwise() - A.colwise().mean(). A is converted to a newA
    myGeneratorPCA.calculatePCAbySVD(0,BB, BBpca, pcaB);//B is converted to PCA. Important,inside this function also is calculated B.rowwise() - B.colwise().mean(). B is converted to a newB
    myGeneratorPCA.setPcaA(pcaA);
    myGeneratorPCA.setPcaB(pcaB);
    statusBar()->showMessage(tr("Please wait while calculating ..."));
    statusProgressBar->setValue(10);

    Vector3d myScalaSVD = myFindScala.getScalaSVD(AApca,BBpca);
    //Vector3d myScalaEigen = myFindScala.getScalaEigenValues(AApca,BBpca);
    std::cout <<"MainWindow::onEstimateSequence myScalaSVD="<<myScalaSVD<<std::endl;
    //std::cout <<"MainWindow::onEstimateSequence myScalaEigen="<<myScalaEigen<<std::endl;
    //Vector3d myScalaSVD2 = myFindScala.getScalaSVD(AA,BB);
    //Vector3d myScalaEigen2 = myFindScala.getScalaEigenValues(AA,BB);
    //std::cout <<"MainWindow::onEstimateSequence myScalaSVD2="<<myScalaSVD2<<std::endl;
    //std::cout <<"MainWindow::onEstimateSequence myScalaEigen2="<<myScalaEigen2<<std::endl;

    //Vector3d myScalaVector = myFindScala.getScalaByDivision(AA,BB);
    //std::cout <<"MainWindow::onEstimateSequence myScalaVector="<<myScalaVector<<std::endl;
    //Vector3d myScalaMean = myFindScala.getScalaByMean(AA,BB);
    //Vector3d myScalaMean = myFindScala.getScalaByMean(AApca,BBpca);
    //std::cout <<"MainWindow::onEstimateSequence myScalaVector="<<myScalaMean<<std::endl;


    // End Try to estimate Scala using PCA
    //==========================================

    // Try to find Time offset
    //==========================================
    //double offsetEstimated = 0;
    //if (timeOffset > 0) {
            int maxLine = readingA.rows();
            int interval = 50;
            //double anOffset = timeOffset;
            double correlationValue = 0;
            //double correlationValue = myFindOffset.calculateOffsetXYZ( maxLine,interval, anOffset,offsetEstimated, AApca,  BBpca);//ok
            //timeOffsetEstimated= offsetEstimated;
            //std::cout<<"timeOffsetEstimated="<<timeOffsetEstimated<<std::endl;

        // Calculate interpolation with AApca and BBpca
        // But AApca and BBpca has only 3 coordinates, needs to add the 4th coordinate , which is TIME

        //anOffset=0;
        MatrixXd tAApca(contLinA,4),tBBpca (contLinB,4);// store aapca and bbpca with time
        for (int i =0; i < contLinA; i ++){
            std::cout<<"inside the for i="<<i<<std::endl;
            //tAApca(i,0)=AA(i,0);
            tAApca(i,0)=timeA[i];
            tAApca(i,1)=AApca(i,0);
            tAApca(i,2)=AApca(i,1);
            tAApca(i,3)=AApca(i,2);

        }
        for (int i =0; i < contLinB; i ++){
            std::cout<<"inside the for i="<<i<<std::endl;
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
        statusBar()->showMessage(tr("Please wait while calculating ..."));
        statusProgressBar->setValue(40);
        //timeOffsetEstimated=myInterpolator.calculateOffsetWithInterpolation3( tAApca,  tBBpca);
        std::cout<<"timeOffsetEstimated2="<<timeOffsetEstimated<<std::endl;
        std::cout<<"timeOffset EXPECTED="<<timeOffset<<std::endl;
        std::cout<<"rMax="<<rMax<<std::endl;

        //Apply estimated offset over B
        for (int i=0; i< tBBpca.rows(); i++){
            Vector4d myNewVector(((tBBpca.row(i))(0) + timeOffsetEstimated),(tBBpca.row(i))(1),(tBBpca.row(i))(2),(tBBpca.row(i))(3));
            //Vector4d myNewVector(((tBBpca.row(i))(0) + offsetEstimated),(tBBpca.row(i))(1),(tBBpca.row(i))(2),(tBBpca.row(i))(3));
            //contAddedValues++;



            tBBpca.row(i)<<myNewVector.transpose();
         }
         //myInterpolator.interpolateSerieToFrequency2(frequency, tAApca);
        //Check if the time is aligned for A and B
         for (int t=0; t<10; t++){
                std::cout<<"================================================================timeA["<<t<<"]="<<tAApca(t,0)<<"  timeB["<<t<<"]="<<tBBpca(t,0)<<std::endl;
         }

    //} else { // from if (timeOffset > 0) {
    //     timeOffsetEstimated=0;
    //     rMax=0;
    //}

   // Correct offset of dataB

    MatrixXd dataA(contLinA,8),dataB (contLinB,8);// store dataA and dataB time
    for (int i =0; i < contLinA; i ++){
        std::cout<<"inside the for i="<<i<<std::endl;

        dataA(i,0)=timeA[i];
        dataA(i,1)=readingAbkp(i,0);
        dataA(i,2)=readingAbkp(i,1);
        dataA(i,3)=readingAbkp(i,2);
        dataA(i,4)=readingAbkp(i,3);
        dataA(i,5)=readingAbkp(i,4);
        dataA(i,6)=readingAbkp(i,5);
        dataA(i,7)=readingAbkp(i,6);


    }
    for (int i =0; i < contLinB; i ++){
        std::cout<<"inside the for i="<<i<<std::endl;



        dataB(i,0)=timeB[i]+timeOffsetEstimated;
        dataB(i,1)=readingBbkp(i,0);
        dataB(i,2)=readingBbkp(i,1);
        dataB(i,3)=readingBbkp(i,2);
        dataB(i,4)=readingBbkp(i,3);
        dataB(i,5)=readingBbkp(i,4);
        dataB(i,6)=readingBbkp(i,5);
        dataB(i,7)=readingBbkp(i,6);

    }
    //std::cout<< "MainWindow::offsetEstimated=== ="<< offsetEstimated <<std::endl;

    std::cout<< "MainWindow::offsetEstimated=== ="<< timeOffsetEstimated <<std::endl;
//    for (int i=0; i< dataB.rows(); i++){
//        //Vector4d myNewVector(((dataB.row(i))(0) + offsetEstimated),(dataB.row(i))(1),(dataB.row(i))(2),(dataB.row(i))(3));
//        Vector4d myNewVector(((dataB.row(i))(0) + timeOffsetEstimated),(dataB.row(i))(1),(dataB.row(i))(2),(dataB.row(i))(3));
//        //contAddedValues++;
//        dataB.row(i)<<myNewVector.transpose();
//     }
//just temporal
//ftype=2;
//frequency=0.005;

if (dataSetB_isLoaded_NotTransformed ) {
    //Estimating 2 data sets , A and B loaded from file. In other case dataset B is the result obtained by transformations over dataset A
    double myfreqencyA = myInterpolator.findFrequency(dataA);
    double myfreqencyB = myInterpolator.findFrequency(dataB);
    myfreqencyA = long(myfreqencyA*1000) / 1000.0;
    myfreqencyB = long(myfreqencyB*1000) / 1000.0;
    std::cout<< "MainWindow::myfreqencyA="<< myfreqencyA <<std::endl;
    std::cout<< "MainWindow::myfreqencyB="<< myfreqencyB <<std::endl;
    if (myfreqencyA < myfreqencyB) {
        frequency = myfreqencyA;
    } else {
        frequency = myfreqencyB;

    }

    ftype=2;

}


myInterpolator.performInterpolation(ftype,frequency,dataA,dataB);
std::cout<< "MainWindow::dataA rows="<< dataA.rows() <<" "<< dataA.cols()<<std::endl;
std::cout<< "MainWindow::dataB rows="<< dataB.rows() <<" "<< dataB.cols()<<std::endl;
MatrixXd newDataA,newDataB,dataApca,dataBpca;
newDataA = dataA.block(0,1,dataA.rows(),3);
newDataB = dataB.block(0,1,dataB.rows(),3);
myGeneratorPCA.calculatePCAbySVD(0,newDataA, dataApca, pcaA);//A is converted to PCA. Important,inside this function also is calculated A.rowwise() - A.colwise().mean(). A is converted to a newA
myGeneratorPCA.calculatePCAbySVD(0,newDataB, dataBpca, pcaB);//B is converted to PCA. Important,inside this function also is calculated B.rowwise() - B.colwise().mean(). B is converted to a newB
myGeneratorPCA.setPcaA(pcaA);
myGeneratorPCA.setPcaB(pcaB);
statusBar()->showMessage(tr("Please wait while calculating ..."));
statusProgressBar->setValue(10);

myScalaSVD = myFindScala.getScalaSVD(dataApca,dataBpca);
statusBar()->showMessage(tr("Please wait while calculating ..."));
statusProgressBar->setValue(75);

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
    dataBquaternion = dataB.block(0,4,numRows,4);
    dataAquaternion = dataA.block(0,4,numRows,4);


    // Adapt dataB to Scale if necessary
    //==========================================
    double medScala;//To store the medium value of the estimated scale
    if (myScalaSVD(0) > 1 || myScalaSVD(1) > 1 || myScalaSVD(2) > 1 ){ //if myScaleSVD is not (1,1,1)
        medScala = (myScalaSVD(0)+myScalaSVD(1)+myScalaSVD(2))/3.0;
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
            if (RANSAC)
                myRegistrador.applyRANSAC(readingA,newB,rotationEstimated,traslationEstimated);
            else
                myRegistrador.rigid_transform_3D(readingA,newB,rotationEstimated,traslationEstimated);
            std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
            std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
            myRegistrador.applyTransformationsOverData(readingA,dataEstimated,rotationEstimated,traslationEstimated);

            myRegistrador.applyTransformationsOverQuaternion(dataAquaternion,dataQuaternionEstimated,rotationEstimated);
            MatrixXd newA(dataEstimated.rows(),3);
            newA = dataEstimated.block(0,0,dataEstimated.rows(),3);

            for (int i= 0; i< newA.rows(); i++){
                VectorXd aRow = newA.row(i);
                newA.row(i)<< aRow(0)*medScala,aRow(1)*medScala,aRow(2)*medScala;
             }
             dataEstimated = newA.block(0,0,newA.rows(),3);
            //myRegistrador.applyTransformationsOverData(newA,dataEstimated,rotationEstimated,traslationEstimated);


        } else { //B to A
            if (RANSAC)
                myRegistrador.applyRANSAC(newB,readingA,rotationEstimated,traslationEstimated);
            else
                myRegistrador.rigid_transform_3D(newB,readingA,rotationEstimated,traslationEstimated);
            //myRegistrador.rigid_transform_3D(readingA,newB,rotationEstimated,traslationEstimated);
            std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
            std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
            myRegistrador.applyTransformationsOverData(newB,dataEstimated,rotationEstimated,traslationEstimated);
            myRegistrador.applyTransformationsOverQuaternion(dataBquaternion,dataQuaternionEstimated,rotationEstimated);
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
        medScala = (myScalaSVD(0)+myScalaSVD(1)+myScalaSVD(2))/3.0;
        if (way == 0) { // A to B
        //myRegistrador.rigid_transform_3D(readingB,readingA,rotationEstimated,traslationEstimated);
        myRegistrador.rigid_transform_3D(readingA,readingB,rotationEstimated,traslationEstimated);
        std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
        std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
        //myRegistrador.applyTransformationsOverData(readingB,dataEstimated,rotationEstimated,traslationEstimated);
        myRegistrador.applyTransformationsOverData(readingA,dataEstimated,rotationEstimated,traslationEstimated);
        myRegistrador.applyTransformationsOverQuaternion(dataAquaternion,dataQuaternionEstimated,rotationEstimated);

        } else { // B to A

        myRegistrador.rigid_transform_3D(readingB,readingA,rotationEstimated,traslationEstimated);
        std::cout<< "MainWindow::onEstimateSequence rotationEstimated="<<rotationEstimated <<std::endl;
        std::cout<< "MainWindow::onEstimateSequence traslationEstimated="<<traslationEstimated <<std::endl;
        myRegistrador.applyTransformationsOverData(readingB,dataEstimated,rotationEstimated,traslationEstimated);
        myRegistrador.applyTransformationsOverQuaternion(dataBquaternion,dataQuaternionEstimated,rotationEstimated);

        }
    }

    if (!dataSetB_isLoaded_NotTransformed) { //if datasetB is the result of transforming datasetA
        Quaterniond q(myTransformador.getMatRot_toQuaternion());
        Quaterniond p(myRegistrador.getMatRot_toQuaternion());

        q.normalize();
        p.normalize();
        std::cout<<"qwxyz="<<q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<std::endl;
        std::cout<<"pwxyz="<<p.w()<<" "<<p.x()<<" "<<p.y()<<" "<<p.z()<<std::endl;
        //std::cout<<"quat_FromRotEstimatedwxyz="<<quat_FromRotEstimated.w()<<" "<<quat_FromRotEstimated.x()<<" "<<quat_FromRotEstimated.y()<<" "<<quat_FromRotEstimated.z()<<std::endl;
        float myAngularDistance=p.angularDistance(q);
        std::cout<<"MyAngularDistance="<<myAngularDistance<<std::endl;
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
    //myMessage->setVisible(false);
    //myMessage->close();
    //msgBox.setVisible(false);
    //msgBox.close();
    statusBar()->showMessage(tr("Please wait while calculating ..."));
    statusProgressBar->setValue(100);
    dialogShowEstimated->setDataDialog(dataDialogShowEstimated);
    dialogShowEstimated->show();
    //dialogMessage->show();

    // write to file dataEstimated
    std::ofstream out("/tmp/miOutputEstimated.txt" );
    //out << std::setprecision(6) << std::fixed;
    int rows = dataEstimated.rows();
    for (int i=0;i<rows;i++){
        //MISSING TIME !!
        out<< dataEstimated(i,0) <<' '<<dataEstimated(i,1)<<' '<<dataEstimated(i,2)<<' '<<dataQuaternionEstimated(i,0)<<' '<<dataQuaternionEstimated(i,1)<<' '<<dataQuaternionEstimated(i,2)<<' '<<dataQuaternionEstimated(i,3)<<'\n';
    }
    out.close();

}
//================================================================================================================
void MainWindow::onEstimateSequenceAtoB(){

    //
    //this->setTextOnStatusBar();
    statusBar()->showMessage(tr("Please wait while calculating ..."));
    //dialogMessage->update();
    //dialogMessage->show();
    //dialogMessage->repaint();
    //update();

    //QThread::sleep(5);
    bool RANSAC=false;
    onEstimateSequence(0,RANSAC);
    statusBar()->showMessage(tr("Estimated secuence calculated..."));
    statusProgressBar->setValue(0);
    //this->setTextOnStatusBar("Estimated secuence calculated");
    //dialogMessage->close();
}

void MainWindow::onEstimateSequenceBtoA(){//estimate transformations from dataset B to dataset A
    statusBar()->showMessage(tr("Please wait while calculating ..."));
    bool RANSAC=false;
    onEstimateSequence(1,RANSAC);
    statusProgressBar->setValue(0);
    statusBar()->showMessage(tr("Estimated secuence calculated..."));
}

void MainWindow::onEstimateSequenceAtoB_RANSAC(){

    //
    //this->setTextOnStatusBar();
    statusBar()->showMessage(tr("Please wait while calculating ..."));
    //dialogMessage->update();
    //dialogMessage->show();
    //dialogMessage->repaint();
    //update();

    //QThread::sleep(5);
    bool RANSAC=true;
    onEstimateSequence(3,RANSAC);
    statusBar()->showMessage(tr("Estimated secuence calculated, A to B, using RANSAC..."));
    statusProgressBar->setValue(0);
    //this->setTextOnStatusBar("Estimated secuence calculated");
    //dialogMessage->close();
}

void MainWindow::onEstimateSequenceBtoA_RANSAC(){//estimate transformations from dataset B to dataset A
    statusBar()->showMessage(tr("Please wait while calculating ..."));
    bool RANSAC=true;
    onEstimateSequence(4,RANSAC);
    statusProgressBar->setValue(0);
    statusBar()->showMessage(tr("Estimated secuence calculated, B TO A , using RANSAC..."));
}

void MainWindow::onModifyParameters(){
//    dialogConfiguration = (new DialogConfiguration(this));
//    dataDialogConfiguration = new DataDialogConfiguration(50000,3,0.01);
//    dialogConfiguration->setDataDialog(dataDialogConfiguration);
//    dialogConfiguration->show();
      dialogParameters = (new DialogParameters(this));
      //dataDialogParameters = new DataDialogParameters(50000,0.01,3);
      dialogParameters->setDataDialog(dataDialogParameters);
      update();
      dialogParameters->show();


}

//============================================================================================================
void MainWindow::loadFile(const QString &fileName,MatrixXd &dataSet,int dataSet_A_B)
//fileName: is the file name to read
//dataSet: is the output of the dataset
//tipeA_B: is an integer 0 or 1, 0 means type A (groundtruth) , 1 means type B (secondary datafile)
{
    //int MAX_ROWS=50000;
    int MAX_ROWS=dataDialogParameters->getMAXLINES();
    VectorXd myTimeA(MAX_ROWS);
    VectorXd myTimeB(MAX_ROWS);
    QFile file(fileName);
    //int maxLine = 10000;

    MatrixXd newA(MAX_ROWS,7);
    //Eigen::MatrixXd readingA (maxLine,3);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, tr("Application"),
                             tr("Cannot read file %1:\n%2.")
                             .arg(QDir::toNativeSeparators(fileName), file.errorString()));
        return ;
    } else{
        std::cout<< "leyendo archivo" <<std::endl;
        std::ifstream infileA( fileName.toStdString());
        double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
        int contLin=0;


        //while ( (infileA >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
        while ( (infileA >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) ){
                    std::cout <<"contLin="<<contLin<<std::endl;

                    if (contLin >=MAX_ROWS){
                        std::cout<<" NUMBER OF MAXIMUM ROWS EXCEEDED . ONLY READING MAXIMUM ROWS 10000";
                        break;
                    }

                    newA.row(contLin)<< rx,ry,rz,q1,q2,q3,q4;
                    if (dataSet_A_B == 0 ){
                        myTimeA[contLin]= timestamp;//timestamp column of dataset A
                    }else {
                        myTimeB[contLin]= timestamp;//timestamp column of dataset B
                    }
                    std::cout <<"contLin="<<contLin<<std::endl;
                    contLin ++;


        }

        dataSet =newA.block(0,0,contLin,7);
        timeA = myTimeA.block(0,0,MAX_ROWS,1);
        timeB = myTimeB.block(0,0,MAX_ROWS,1);
        //readingA=newA.block(0,0,contLin,3);
        //readingAbkp=newA.block(0,0,contLin,3);
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

    //((Winslam*)(myWinSlam))->setDataView(readingA);
    //return readingA;
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


     //int MAX_ROWS = 50000;
     int MAX_ROWS=dataDialogParameters->getMAXLINES();
     Point3D  myScala;
     myScala.setXYZ(scalaX,scalaY,scalaZ);

     Point3D miTraslacion ;
     miTraslacion.setXYZ(traslaX,traslaY,traslaZ);

     myOutputFileName="/tmp/miSalidaContaminadaQT.txt";
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
     std::ifstream infileB( "/tmp/miSalidaContaminadaQT.txt" );
     //Eigen::MatrixXd readingB (maxLine,3);
     double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
     int contLin=0;

     MatrixXd newB(MAX_ROWS,7);
     while ( (infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<MAX_ROWS ){
                 //std::cout <<"contLin="<<contLin<<std::endl;
                 newB.row(contLin)<< rx,ry,rz,q1,q2,q3,q4;
                 timeB[contLin]= timestamp;
                 std::cout <<"contLin="<<contLin<<std::endl;
                 contLin ++;


     }


     readingB=newB.block(0,0,contLin,7);
     readingBbkp=newB.block(0,0,contLin,7);
     infileB.close();
     ((Winslam*)(myWinSlam))->setContaminatedDataView(readingB);
     timeOffset=offset;
     frequency=freq;
     ftype=fType;
     dataSetB_isLoaded_NotTransformed=false;
}

void MainWindow::setTextOnStatusBar(QString aText){
    statusLabel->setText("Status Bar:"+aText);
    this->update();
    this->repaint();
}


//void MainWindow::setMaxLines(int numLines){
//    MAXLINES=numLines;
//}
//void MainWindow::setOffsetStep(double step){
//    offsetStep=step;
//}
//void MainWindow::setOffsetWindow(double windowSize){
//    offsetWindow=windowSize;
//}

//int MainWindow::getMaxLines(){
//   return MAXLINES;
//}
//double MainWindow::getOffsetStep(){
//   return  offsetStep;
//}
//double MainWindow::getOffsetWindow(){
//   return  offsetWindow;
//}
void MainWindow::cleanDataSets(){
    MatrixXd cleanMatrix = MatrixXd::Zero(1, 7);
    readingA=cleanMatrix.block(0,0,1,7);
    readingAbkp=cleanMatrix.block(0,0,1,7);
    readingBbkp=cleanMatrix.block(0,0,1,7);
    readingB=cleanMatrix.block(0,0,1,7);
    dataEstimated=cleanMatrix.block(0,0,1,7);

    ((Winslam*)(myWinSlam))->setDataView(readingA);

    ((Winslam*)(myWinSlam))->setContaminatedDataView(readingB);

    ((Winslam*)(myWinSlam))->setEstimatedDataView(dataEstimated);
    update();
}
