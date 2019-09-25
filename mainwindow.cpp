#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  //Configuration myConfiguration;
  Configuration::setMaxLines(500000);
  Configuration::setStepOffset(1 / 100.0);
  Configuration::setWindowOffset(3);
  int maxLine = 3000;
  Eigen::MatrixXd newMatrixA(maxLine, 3);
  readingA = newMatrixA;
  Eigen::MatrixXd newMatrixB(maxLine, 3);
  readingB = newMatrixB;
  Eigen::MatrixXd newMatrixEstimated(maxLine, 3);
  dataEstimated = newMatrixEstimated;

  connect(ui->openFileA, &QAction::triggered, this, &MainWindow::onOpenFileA);
  connect(ui->openFileB, &QAction::triggered, this, &MainWindow::onOpenFileB);
  connect(ui->exitApp, &QAction::triggered, this, &MainWindow::onExit);
  connect(ui->modifySequence, &QAction::triggered, this, &MainWindow::onModifySequence);
  connect(ui->estimateSequenceAtoB, &QAction::triggered, this, &MainWindow::onEstimateSequenceAtoB);
  connect(ui->estimateSequenceBtoA, &QAction::triggered, this, &MainWindow::onEstimateSequenceBtoA);
  connect(ui->estimateSequenceAtoB_RANSAC, &QAction::triggered, this, &MainWindow::onEstimateSequenceAtoB_RANSAC);
  connect(ui->estimateSequenceBtoA_RANSAC, &QAction::triggered, this, &MainWindow::onEstimateSequenceBtoA_RANSAC);
  connect(ui->setDots, &QAction::triggered, this, &MainWindow::onSetDots);
  connect(ui->setLines, &QAction::triggered, this, &MainWindow::onSetLines);
  connect(ui->setViewJustEstimated, &QAction::triggered, this, &MainWindow::onViewJustEstimated);
  connect(ui->setConfParameters, &QAction::triggered, this, &MainWindow::onModifyParameters);

  statusLabel = new QLabel("");
  statusProgressBar = new QProgressBar(this);

  statusBar()->addWidget(statusLabel, 1);

  statusBar()->showMessage(tr("Status Bar"));
  statusBar()->addPermanentWidget(statusProgressBar, 0);
  statusProgressBar->setValue(100);
  statusProgressBar->setValue(0);

  myWinSlam = (QWidget *) (new Winslam(this));
  setCentralWidget(myWinSlam);
  dataDialogScalaTraslaRota =
      new DataDialogScalaTraslaRota(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.005);
  dataDialogParameters = new DataDialogParameters(50000, 0.01, 3);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::loadFile(const QString &fileName, Eigen::MatrixXd &dataSet, int dataSet_A_B)
//fileName: is the file name to read
//dataSet: is the output of the dataset
//tipeA_B: is an integer 0 or 1, 0 means type A (groundtruth) , 1 means type B (secondary datafile)
{
  int MAX_ROWS = dataDialogParameters->getMAXLINES();
  Eigen::MatrixXd temp_mat(MAX_ROWS, 7);
  Eigen::VectorXd myTimeA(MAX_ROWS);
  Eigen::VectorXd myTimeB(MAX_ROWS);
  QFile file(fileName);

  Eigen::MatrixXd newA(MAX_ROWS, 7);

  if (!file.open(QFile::ReadOnly | QFile::Text))
  {
    QMessageBox::warning(this, tr("Application"),
                         tr("Cannot read file %1:\n%2.")
                             .arg(QDir::toNativeSeparators(fileName), file.errorString()));
    return;
  } else
  {

    if (dataSet_A_B == 0)
    {
      std::ifstream infileA(fileName.toStdString());
      double timestamp, rx, ry, rz, q1, q2, q3, q4 = 0;
      int contLin = 0;

      while ((infileA >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4))
      {

        if (contLin >= MAX_ROWS)
        {
          std::cout << " NUMBER OF MAXIMUM ROWS EXCEEDED . ONLY READING MAXIMUM ROWS:" << MAX_ROWS << endl;
          break;
        } else
        {
          temp_mat.row(contLin) << rx, ry, rz, q1, q2, q3, q4;
          myTimeA[contLin] = timestamp;
        }
        contLin++;

      }
      infileA.close();
      dataSet = temp_mat.block(0, 0, contLin, 7);
      timeA = myTimeA.block(0, 0, contLin, 1);
    } else
    {
      std::ifstream infileB(fileName.toStdString());
      double timestamp, rx, ry, rz, q1, q2, q3, q4 = 0;
      int contLin = 0;

      while ((infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4))
      {

        if (contLin >= MAX_ROWS)
        {
          std::cout << " NUMBER OF MAXIMUM ROWS EXCEEDED . ONLY READING MAXIMUM ROWS:" << MAX_ROWS << endl;
          break;
        } else
        {
          temp_mat.row(contLin) << rx, ry, rz, q1, q2, q3, q4;
          myTimeB[contLin] = timestamp;
        }
        contLin++;

      }
      infileB.close();
      dataSet = temp_mat.block(0, 0, contLin, 7);
      timeB = myTimeB.block(0, 0, contLin, 1);
    }

    std::cout << "finish reading file" << std::endl;
  }

  QTextStream in(&file);
#ifndef QT_NO_CURSOR
  QApplication::setOverrideCursor(Qt::WaitCursor);
#endif

#ifndef QT_NO_CURSOR
  QApplication::restoreOverrideCursor();
#endif

}

void MainWindow::performModifySequence(double scalaX,
                                       double scalaY,
                                       double scalaZ,
                                       double traslaX,
                                       double traslaY,
                                       double traslaZ,
                                       double rotaX,
                                       double rotaY,
                                       double rotaZ,
                                       int gNoise,
                                       int cNoise,
                                       double offset,
                                       int fType,
                                       double freq,
                                       double gnoise_value)
{
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
  dataDialogScalaTraslaRota->setTimeOffset(offset);
  dataDialogScalaTraslaRota->setFrequencyType(fType);
  dataDialogScalaTraslaRota->setFrequency(freq);


  //int MAX_ROWS = 50000;
  int MAX_ROWS = dataDialogParameters->getMAXLINES();
  Point3D myScala;
  myScala.setXYZ(scalaX, scalaY, scalaZ);

  Point3D miTraslacion;
  miTraslacion.setXYZ(traslaX, traslaY, traslaZ);

  myOutputFileName = (char *) "/tmp/miSalidaContaminadaQT.txt";
  std::cout << "mainwindow.performModifySequence  gNoise=" << gNoise << " cNoise=" << cNoise << std::endl;

  if ((gNoise > 0.0) & (cNoise > 0.0))
  {
    myTransformador.createContaminatedSequence(myInputFileName,
                                               myOutputFileName,
                                               miTraslacion,
                                               myScala,
                                               rotaX,
                                               rotaY,
                                               rotaZ,
                                               1,
                                               1,
                                               offset,
                                               fType,
                                               freq, gnoise_value);
  } else if ((gNoise > 0.0) & (cNoise <= 0.0))
  {
    myTransformador.createContaminatedSequence(myInputFileName,
                                               myOutputFileName,
                                               miTraslacion,
                                               myScala,
                                               rotaX,
                                               rotaY,
                                               rotaZ,
                                               1,
                                               0,
                                               offset,
                                               fType,
                                               freq, gnoise_value);
  } else if ((gNoise <= 0) & (cNoise > 0.0))
  {
    myTransformador.createContaminatedSequence(myInputFileName,
                                               myOutputFileName,
                                               miTraslacion,
                                               myScala,
                                               rotaX,
                                               rotaY,
                                               rotaZ,
                                               0,
                                               1,
                                               offset,
                                               fType,
                                               freq, gnoise_value);
  } else
    myTransformador.createContaminatedSequence(myInputFileName,
                                               myOutputFileName,
                                               miTraslacion,
                                               myScala,
                                               rotaX,
                                               rotaY,
                                               rotaZ,
                                               0,
                                               0,
                                               offset,
                                               fType,
                                               freq, gnoise_value);


  // Reading input file B, with new dataset contaminated
  std::ifstream infileB("/tmp/miSalidaContaminadaQT.txt");

  double timestamp, rx, ry, rz, q1, q2, q3, q4 = 0;
  int contLin = 0;

  Eigen::MatrixXd newB(MAX_ROWS, 7);
  timeB = VectorXd::Zero(MAX_ROWS);
  while ((infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin < MAX_ROWS)
  {
    newB.row(contLin) << rx, ry, rz, q1, q2, q3, q4;
    timeB[contLin] = timestamp;
    contLin++;

  }

  readingB = newB.block(0, 0, contLin, 7);

  infileB.close();
  ((Winslam *) (myWinSlam))->setContaminatedDataView(readingB);
  timeOffset = offset;
  frequency = freq;
  ftype = fType;
  dataSetB_isLoaded_NotTransformed = false;
}

void MainWindow::cleanDataSets()
{
  Eigen::MatrixXd cleanMatrix = Eigen::MatrixXd::Zero(1, 7);
  readingA = cleanMatrix.block(0, 0, 1, 7);
  readingA = cleanMatrix.block(0, 0, 1, 7);
  readingB = cleanMatrix.block(0, 0, 1, 7);
  readingB = cleanMatrix.block(0, 0, 1, 7);
  dataEstimated = cleanMatrix.block(0, 0, 1, 7);

  ((Winslam *) (myWinSlam))->setDataView(readingA);

  ((Winslam *) (myWinSlam))->setContaminatedDataView(readingB);

  ((Winslam *) (myWinSlam))->setEstimatedDataView(dataEstimated);
  update();
}

void MainWindow::onOpenFileA()
{

  QString fileName = QFileDialog::getOpenFileName(this);
  std::string fname = fileName.toStdString();
  char *cstr = new char[fname.size() + 1];
  strcpy(cstr, fname.c_str());
  myInputFileName = cstr;
  if (!fileName.isEmpty())
  {
    Eigen::MatrixXd newData;
    loadFile(fileName, newData, 0);
    readingA = newData.block(0, 0, newData.rows(), 7);
    ((Winslam *) (myWinSlam))->setDataView(readingA);

  }

}

void MainWindow::onOpenFileB()
{

  QString fileName = QFileDialog::getOpenFileName(this);
  std::string fname = fileName.toStdString();
  char *cstr = new char[fname.size() + 1];
  strcpy(cstr, fname.c_str());
  myInputFileName = cstr;
  if (!fileName.isEmpty())
  {
    Eigen::MatrixXd newData;
    loadFile(fileName, newData, 1);
    readingB = newData.block(0, 0, newData.rows(), 7);
    ((Winslam *) (myWinSlam))->setContaminatedDataView(readingB);

  }
  //to indicate that frequency must be calculated for datasetB
  dataSetB_isLoaded_NotTransformed = true;

}

void MainWindow::onModifySequence()
{
  std::cout << "onModifySequence" << std::endl;
  //dialogScalaTraslaRota =(QDialog*) (new DialogScalaTraslaRota (this));
  dialogScalaTraslaRota = (new DialogScalaTraslaRota(this));
  //dialogScalaTraslaRota->setDataDialog(new DataDialogScalaTraslaRota(1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0));
  dialogScalaTraslaRota->show();
  dialogScalaTraslaRota->setDataDialog(dataDialogScalaTraslaRota);
  if (dataDialogScalaTraslaRota->getFrequencyType() == 0)
  {
    dialogScalaTraslaRota->onPressMaxFrequency();
  } else if (dataDialogScalaTraslaRota->getFrequencyType() == 1)
  {
    dialogScalaTraslaRota->onPressMinFrequency();
  } else if (dataDialogScalaTraslaRota->getFrequencyType() == 2)
  {
    dialogScalaTraslaRota->onPressCustomizedFrequency();
  }
  dialogScalaTraslaRota->update();
  dataDialogScalaTraslaRota = dialogScalaTraslaRota->getDataDialog();

  std::cout << "onModifySequence.dataDialogScalaTraslaRota->getCosmicNoiseDeviation()"
            << dataDialogScalaTraslaRota->getCosmicNoiseDeviation();
  std::cout << "FIN onModifySequence" << std::endl;
}

void MainWindow::onEstimateSequenceAtoB()
{
  statusBar()->showMessage(tr("Please wait while calculating ..."));
  bool RANSAC = false;
  onEstimateSequence(0, RANSAC);
  statusBar()->showMessage(tr("Estimated sequence calculated..."));
  statusProgressBar->setValue(0);

}

void MainWindow::onEstimateSequenceBtoA()
{   //estimate transformations from dataset B to dataset A
  statusBar()->showMessage(tr("Please wait while calculating ..."));
  bool RANSAC = false;
  onEstimateSequence(1, RANSAC);
  statusProgressBar->setValue(0);
  statusBar()->showMessage(tr("Estimated secuence calculated..."));
}

void MainWindow::onEstimateSequenceAtoB_RANSAC()
{

  statusBar()->showMessage(tr("Please wait while calculating ..."));
  bool RANSAC = true;
  onEstimateSequence(2, RANSAC);
  statusBar()->showMessage(tr("Estimated secuence calculated, A to B, using RANSAC..."));
  statusProgressBar->setValue(0);
}

void MainWindow::onEstimateSequenceBtoA_RANSAC()
{//estimate transformations from dataset B to dataset A
  statusBar()->showMessage(tr("Please wait while calculating ..."));
  bool RANSAC = true;
  onEstimateSequence(3, RANSAC);
  statusProgressBar->setValue(0);
  statusBar()->showMessage(tr("Estimated secuence calculated, B TO A , using RANSAC..."));
}

void MainWindow::onModifyParameters()
{

  dialogParameters = (new DialogParameters(this));
  dialogParameters->setDataDialog(dataDialogParameters);
  update();
  dialogParameters->show();

}
void MainWindow::onExit()
{
  this->close();
}
void MainWindow::onSetDots()
{
  ((Winslam *) (myWinSlam))->setDots();
}
void MainWindow::onSetLines()
{
  ((Winslam *) (myWinSlam))->setLines();
}
void MainWindow::onViewJustEstimated()
{
  ((Winslam *) (myWinSlam))->setViewJustEstimated();
}

void MainWindow::onEstimateSequence(int way,
                                    bool RANSAC)
{   //Estimate transformations from dataset A to dataset B or dataset B to dataset A
  float rMax = 0.0;

  readingA_xyz = readingA.block(0, 0, readingA.rows(), 3);//to use with Scale and PCA
  readingB_xyz = readingB.block(0, 0, readingB.rows(), 3);
  int contLinA = readingA_xyz.rows();
  int contLinB = readingB_xyz.rows();

  //Begin Try to estimate Scala using PCA
  //=========================================
  Eigen::MatrixXd AApca(contLinA, 3), BBpca(contLinB, 3);
  Eigen::MatrixXd pcaA, pcaB;

  myGeneratorPCA.calculatePCAbySVD(0,
                                   readingA_xyz,
                                   AApca,
                                   pcaA);//A is converted to PCA. Important,inside this function also is calculated A.rowwise() - A.colwise().mean(). A is converted to a newA
  myGeneratorPCA.calculatePCAbySVD(0,
                                   readingB_xyz,
                                   BBpca,
                                   pcaB);//B is converted to PCA. Important,inside this function also is calculated B.rowwise() - B.colwise().mean(). B is converted to a newB
  myGeneratorPCA.setPcaA(pcaA);
  myGeneratorPCA.setPcaB(pcaB);
  statusBar()->showMessage(tr("Please wait while calculating ..."));
  statusProgressBar->setValue(10);

  Eigen::Vector3d myScalaSVD = myFindScala.getScalaSVD(AApca, BBpca);
  std::cout << "MainWindow::onEstimateSequence myScalaSVD=" << myScalaSVD << std::endl;

  // End Try to estimate Scala using PCA
  //==========================================
  // Try to find Time offset
  // Calculate interpolation with AApca and BBpca
  // But AApca and BBpca has only 3 coordinates, needs to add the 4th coordinate , which is TIME

  Eigen::MatrixXd tAApca(contLinA, 4), tBBpca(contLinB, 4);// store aapca and bbpca with time

  tAApca.block(0, 0, contLinA, 1) = timeA;
  tAApca.block(0, 1, contLinA, 3) = AApca;
  std::cout << timeB << "--------------------" << std::endl;
  timeB = timeB.block(0, 0, contLinB, 1);
  tBBpca.block(0, 0, contLinB, 1) = timeB;
  tBBpca.block(0, 1, contLinB, 3) = BBpca;

  timeOffsetEstimated = myInterpolator.calculateOffsetWithInterpolation2(tAApca, tBBpca, rMax); //Good
  timeOffsetEstimated = int(timeOffsetEstimated * 1000) / 1000.0;
  statusBar()->showMessage(tr("Please wait while calculating ..."));
  statusProgressBar->setValue(40);



  // Correct offset of dataB

  Eigen::MatrixXd dataA(contLinA, 8), dataB(contLinB, 8);// store dataA and dataB time



  dataA.block(0, 0, contLinA, 1) = timeA;
  dataA.block(0, 1, contLinA, 7) = readingA;

  dataB.block(0, 0, contLinB, 1) = timeB;
  dataB.block(0, 1, contLinB, 7) = readingB;

  std::cout << "MainWindow::offsetEstimated=== =" << timeOffsetEstimated << std::endl;

  if (dataSetB_isLoaded_NotTransformed)
  {
    //Estimating 2 data sets , A and B loaded from file. In other case dataset B is the result obtained by transformations over dataset A
    double myfreqencyA = myInterpolator.findFrequency(dataA);
    double myfreqencyB = myInterpolator.findFrequency(dataB);
    myfreqencyA = long(myfreqencyA * 1000) / 1000.0;
    myfreqencyB = long(myfreqencyB * 1000) / 1000.0;
    if (myfreqencyA < myfreqencyB)
    {
      frequency = myfreqencyA;
    } else
    {
      frequency = myfreqencyB;

    }

    ftype = 2;

  }

  myInterpolator.performInterpolation(ftype, frequency, dataA, dataB);


  //Check if the time is aligned for A and B


  int numRows;
  if (dataB.rows() > dataA.rows())
  {
    //after interpolation to fmin it might be possible to adjust number of cols
    numRows = dataA.rows();

  } else numRows = dataB.rows();

  std::cout << "MainWindow::dataA rows=" << dataA.rows() << " " << dataA.cols() << std::endl;
  std::cout << "MainWindow::dataB rows=" << dataB.rows() << " " << dataB.cols() << std::endl;

  readingBquaternion = dataB.block(0, 4, numRows, 4);
  readingAquaternion = dataA.block(0, 4, numRows, 4);


  // Adapt dataB to Scale if necessary
  //==========================================
  double medScala;//To store the medium value of the estimated scale
  if (myScalaSVD(0) > 1 || myScalaSVD(1) > 1 || myScalaSVD(2) > 1)
  {
    //if myScaleSVD is not (1,1,1)
    medScala = (myScalaSVD(0) + myScalaSVD(1) + myScalaSVD(2)) / 3.0;
    MatrixXd newB(readingB_xyz.rows(), 3);
    newB = readingB_xyz.block(0, 0, readingB_xyz.rows(), 3);
    for (int i = 0; i < newB.rows(); i++)
    {
      VectorXd aRow = newB.row(i);
      newB.row(i) << aRow(0) / medScala, aRow(1) / medScala, aRow(2) / medScala;
    }

    if (way == 0 || way == 2)
    {
      //A to B
      if (RANSAC)
        myRegistrador.applyRANSAC(readingA_xyz, newB, rotationEstimated, traslationEstimated);
      else
        myRegistrador.rigid_transform_3D(readingA_xyz, newB, rotationEstimated, traslationEstimated);

      myRegistrador.applyTransformationsOverData(readingA_xyz, dataEstimated, rotationEstimated, traslationEstimated);
      myRegistrador.applyTransformationsOverQuaternion(readingAquaternion, dataQuaternionEstimated, rotationEstimated);
      Eigen::MatrixXd newA(dataEstimated.rows(), 3);
      newA = dataEstimated.block(0, 0, dataEstimated.rows(), 3);

      for (int i = 0; i < newA.rows(); i++)
      {
        Eigen::VectorXd aRow = newA.row(i);
        newA.row(i) << aRow(0) * medScala, aRow(1) * medScala, aRow(2) * medScala;
      }
      dataEstimated = newA.block(0, 0, newA.rows(), 3);

    } else if (way == 1 || way == 3)
    {
      //B to A
      if (RANSAC)
        myRegistrador.applyRANSAC(newB, readingA_xyz, rotationEstimated, traslationEstimated);
      else
        myRegistrador.rigid_transform_3D(newB, readingA_xyz, rotationEstimated, traslationEstimated);
      myRegistrador.applyTransformationsOverData(newB, dataEstimated, rotationEstimated, traslationEstimated);
      myRegistrador.applyTransformationsOverQuaternion(readingBquaternion, dataQuaternionEstimated, rotationEstimated);
    }

  } else
  {
    medScala = (myScalaSVD(0) + myScalaSVD(1) + myScalaSVD(2)) / 3.0;
    if (way == 0 || way == 2)
    { // A to B
      if (RANSAC)
        myRegistrador.applyRANSAC(readingA_xyz, readingB_xyz, rotationEstimated, traslationEstimated);
      else
        myRegistrador.rigid_transform_3D(readingA_xyz, readingB_xyz, rotationEstimated, traslationEstimated);
      myRegistrador.applyTransformationsOverData(readingA_xyz, dataEstimated, rotationEstimated, traslationEstimated);
      myRegistrador.applyTransformationsOverQuaternion(readingAquaternion, dataQuaternionEstimated, rotationEstimated);

    } else if (way == 1 || way == 3)
    { // B to A
      if (RANSAC)
        myRegistrador.applyRANSAC(readingB_xyz, readingA_xyz, rotationEstimated, traslationEstimated);
      else
        myRegistrador.rigid_transform_3D(readingB_xyz, readingA_xyz, rotationEstimated, traslationEstimated);
      myRegistrador.applyTransformationsOverData(readingB_xyz, dataEstimated, rotationEstimated, traslationEstimated);
      myRegistrador.applyTransformationsOverQuaternion(readingBquaternion, dataQuaternionEstimated, rotationEstimated);

    }
  }

  if (dataSetB_isLoaded_NotTransformed)
  {         //if datasetB is loaded from file
    Eigen::Quaterniond q(1, 0, 0, 0);
    Eigen::Quaterniond p(myRegistrador.getMatRot_toQuaternion());

    q.normalize();
    p.normalize();
    float myAngularDistance = p.angularDistance(q);

  } else
  {
    //if datasetB is the result of transforming datasetA

    Eigen::Quaterniond q(myTransformador.getMatRot_toQuaternion());
    Eigen::Quaterniond p(myRegistrador.getMatRot_toQuaternion());

    q.normalize();
    p.normalize();
    float myAngularDistance = p.angularDistance(q);
  }
  std::cout << "TRANSLATION ESTIMATED----------------------------->" << traslationEstimated << std::endl;

  ((Winslam *) (myWinSlam))->setEstimatedDataView(dataEstimated);
  dialogShowEstimated = (new DialogShowEstimated(this));
  double x1 = rotationEstimated.row(0)(0);
  double y1 = rotationEstimated.row(0)(1);
  double z1 = rotationEstimated.row(0)(2);

  double x2 = rotationEstimated.row(1)(0);
  double y2 = rotationEstimated.row(1)(1);
  double z2 = rotationEstimated.row(1)(2);

  double x3 = rotationEstimated.row(2)(0);
  double y3 = rotationEstimated.row(2)(1);
  double z3 = rotationEstimated.row(2)(2);
  Statistics *myStatistics;
  if (way == 0)
    myStatistics = new Statistics(readingB_xyz, dataEstimated);
  else
    myStatistics = new Statistics(readingA_xyz, dataEstimated);

  double RMSE = myStatistics->RMSE(myStatistics->getErrorRows());
  std::cout << "MainWindow:: Statistics----->> R M S E = " << RMSE << std::endl;
  std::cout << "MainWindow:: lines readingB---->> = " << readingB_xyz.rows() << std::endl;
  std::cout << "MainWindow:: dataEstimated----->> = " << dataEstimated.rows() << std::endl;
  //Calculate Yaw Pitch and Roll from the Rotation Matrix
  Matrix3d myRotationEstimated(3, 3);
  myRotationEstimated = rotationEstimated.block(0, 0, 3, 3);
  Eigen::Matrix<double, 3, 1> ypr = myRotationEstimated.eulerAngles(2, 1, 0);
  double y = ypr(0);//yaw
  double p = ypr(1);//pitch
  double r = ypr(2);//roll
  std::cout << "rpy=====================================================================" << ypr << std::endl;

  dataDialogShowEstimated = new DataDialogShowEstimated(medScala,
                                                        medScala,
                                                        medScala,
                                                        traslationEstimated(0),
                                                        traslationEstimated(1),
                                                        traslationEstimated(2),
                                                        y,
                                                        p,
                                                        r,
                                                        timeOffsetEstimated,
                                                        rMax,
                                                        way,
                                                        RMSE);
  statusBar()->showMessage(tr("Please wait while calculating ..."));
  statusProgressBar->setValue(100);
  dialogShowEstimated->setDataDialog(dataDialogShowEstimated);
  dialogShowEstimated->show();

  // write to file dataEstimated
  std::ofstream out("/tmp/miOutputEstimated.txt");
  int rows = dataEstimated.rows();
  for (int i = 0; i < rows; i++)
  {
    //MISSING TIME !!
    out << dataEstimated(i, 0) << ' ' << dataEstimated(i, 1) << ' ' << dataEstimated(i, 2) << ' '
        << dataQuaternionEstimated(i, 0) << ' ' << dataQuaternionEstimated(i, 1) << ' ' << dataQuaternionEstimated(i, 2)
        << ' ' << dataQuaternionEstimated(i, 3) << '\n';
  }
  out.close();

}