
#include "dialogscalatraslarota.h"
#include "mainwindow.h"

DialogScalaTraslaRota::DialogScalaTraslaRota(QWidget *parent)
    : QDialog(parent)
{
  this->parent=(MainWindow *)parent;
  QDesktopWidget dw;
  int x=dw.width()*0.8;
  int y=dw.height()*0.8;
  QSize newSize( x, y );
  this->resize(newSize);
  std::cout<< "constructor DialogScalaTraslaRota" <<std::endl;
  label = new QLabel(tr("Configure Modifications on data sequence:"));
  //lineEdit = new QLineEdit;
  //label->setBuddy(lineEdit);

  //scaleCheckBox = new QCheckBox(tr("Scale"));
  labScaleX = new QLabel(tr("SCALE X Y Z "));
  scaleX = new QLineEdit;
  scaleX->setText("0");
  scaleX->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  scaleX->setMaximumWidth(100);
  //labScaleY = new QLabel(tr("scale Y"));
  scaleY = new QLineEdit;
  scaleY->setText("0");
  scaleY->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  scaleY->setMaximumWidth(100);
  //labScaleZ = new QLabel(tr("scale X"));
  scaleZ = new QLineEdit;
  scaleZ->setText("0");
  scaleZ->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  scaleZ->setMaximumWidth(100);


  QGroupBox *traslationGroupBox = new QGroupBox(tr("TRASLATION"));
  QVBoxLayout *traslationVBox = new QVBoxLayout;
  //traslaCheckBox = new QCheckBox(tr("TRASLATION"));
  labTraslaX = new QLabel(tr("X"));
  traslaX = new QLineEdit;
  traslaX->setText("0");
  traslaX->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  traslaX->setMaximumWidth(100);
  labTraslaY = new QLabel(tr("Y"));
  traslaY = new QLineEdit;
  traslaY->setText("0");
  traslaY->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  traslaY->setMaximumWidth(100);
  labTraslaZ = new QLabel(tr("Z"));
  traslaZ = new QLineEdit;
  traslaZ->setText("0");
  traslaZ->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  traslaZ->setMaximumWidth(100);

  traslationVBox->addWidget(labTraslaX);
  traslationVBox->addWidget(traslaX);
  traslationVBox->addWidget(labTraslaY);
  traslationVBox->addWidget(traslaY);
  traslationVBox->addWidget(labTraslaZ);
  traslationVBox->addWidget(traslaZ);
  traslationVBox->addStretch(1);
  traslationGroupBox->setLayout(traslationVBox);



  labOffset = new QLabel(tr("Time Offset"));
  timeOffset = new QLineEdit;
  timeOffset->setText("0");
  timeOffset->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  timeOffset->setMaximumWidth(100);
//    labPCAIndex = new QLabel(tr("PCA index"));
//    pcaIndex=new QLineEdit;
//    pcaIndex->setText("0");
//    pcaIndex->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
//    pcaIndex->setMaximumWidth(100);
  /*traslaCheckBox = new QCheckBox(tr("Traslation"));
  traslaX = new QLineEdit;
  traslaX->setText("0");
  traslaY = new QLineEdit;
  traslaY->setText("0");
  traslaZ = new QLineEdit;
  traslaZ->setText("0");
  */
  QGroupBox *rotationGroupBox = new QGroupBox(tr("ROTATION in Radians"));
  QVBoxLayout *rotateVBox = new QVBoxLayout;
  //rotaCheckBox = new QCheckBox(tr("Rotation"));
  labRotaX = new QLabel(tr("X"));
  rotaX = new QLineEdit;
  rotaX->setText("0");
  rotaX->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  rotaX->setMaximumWidth(100);
  labRotaY = new QLabel(tr("Y"));
  rotaY = new QLineEdit;
  rotaY->setText("0");
  rotaY->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  rotaY->setMaximumWidth(100);
  labRotaZ = new QLabel(tr("Z"));
  rotaZ = new QLineEdit;
  rotaZ->setText("0");
  rotaZ->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  rotaZ->setMaximumWidth(100);

  rotateVBox->addWidget(labRotaX);
  rotateVBox->addWidget(rotaX);
  rotateVBox->addWidget(labRotaY);
  rotateVBox->addWidget(rotaY);
  rotateVBox->addWidget(labRotaZ);
  rotateVBox->addWidget(rotaZ);
  rotateVBox->addStretch(1);
  rotationGroupBox->setLayout(rotateVBox);
  label2 = new QLabel(tr("Configure Gaussian and Cosmic Noise:"));
  labelGaussNoise = new QLabel(tr("Gaussian Noise 1/0"));
  gaussianNoiseDeviation = new QLineEdit;
  gaussianNoiseDeviation->setText("0");
  labelCosmicNoise = new QLabel(tr("Cosmic Noise 1/0"));
  cosmicNoiseDeviation = new QLineEdit;
  cosmicNoiseDeviation->setText("0");

  gaussianNoiseCheck = new QCheckBox();
  gaussianNoiseCheck->setChecked(false);
  cosmicNoiseCheck = new QCheckBox();
  cosmicNoiseCheck->setChecked(false);
  QGroupBox *frequencyGroupBox=new QGroupBox(tr("Interpolate FREQUENCY "));
  QVBoxLayout *frequencyVBox = new QVBoxLayout;

  freqMaxRadioButton= new QRadioButton(tr("interpolate to Max Frequency"));

  freqMinRadioButton= new QRadioButton(tr("interpolate to Min Frequency"));

  freqCustomizedRadioButton= new QRadioButton(tr("interpolate to Custom Frequency"));
  //freqMaxRadioButton->setChecked(true);
  freqCustomizedRadioButton->setChecked(true);
  labFrequency = new QLabel(tr("Custom Frequency "));
  frequency = new QLineEdit;
  frequency->setText("0.0");
  frequency->setDisabled(true);

  frequencyVBox->addWidget(freqMaxRadioButton);
  frequencyVBox->addWidget(freqMinRadioButton);
  frequencyVBox->addWidget(freqCustomizedRadioButton);
  frequencyVBox->addWidget(labFrequency);
  frequencyVBox->addWidget(frequency);
  frequencyGroupBox->setLayout(frequencyVBox);





  buttonOK = new QPushButton(tr("&OK"));

  buttonCancel = new QPushButton(tr("&Cancel"));
  std::cout<< "constructor dialogScalaTraslaRota after Button Cancel" <<std::endl;

  connect(buttonOK, SIGNAL(clicked()), this, SLOT(onOK()));
  connect(buttonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
  connect(freqCustomizedRadioButton,SIGNAL(clicked()),this,SLOT(onPressCustomizedFrequency()));
  connect(freqMaxRadioButton,SIGNAL(clicked()),this,SLOT(onPressMaxFrequency()));
  connect(freqMinRadioButton,SIGNAL(clicked()),this,SLOT(onPressMinFrequency()));

  //fromStartCheckBox = new QCheckBox(tr("Search from &start"));
  //fromStartCheckBox->setChecked(true);

  //findButton = new QPushButton(tr("&Find"));
  //findButton->setDefault(true);

  //moreButton = new QPushButton(tr("&More"));
  //moreButton->setCheckable(true);
  //moreButton->setAutoDefault(false);


  //extension = new QWidget;

  //wholeWordsCheckBox = new QCheckBox(tr("&Whole words"));
  //backwardCheckBox = new QCheckBox(tr("Search &backward"));
  //searchSelectionCheckBox = new QCheckBox(tr("Search se&lection"));

  //buttonBox = new QDialogButtonBox(Qt::Vertical);
  //buttonBox->addButton(findButton, QDialogButtonBox::ActionRole);
  //buttonBox->addButton(moreButton, QDialogButtonBox::ActionRole);

  //connect(moreButton, &QAbstractButton::toggled, extension, &QWidget::setVisible);

  //QVBoxLayout *extensionLayout = new QVBoxLayout;
  //extensionLayout->setMargin(0);
  //extensionLayout->addWidget(wholeWordsCheckBox);
  //extensionLayout->addWidget(backwardCheckBox);
  //extensionLayout->addWidget(searchSelectionCheckBox);
  //extension->setLayout(extensionLayout);

  std::cout<< "constructor dialogTraslaRota beforeLayout" <<std::endl;

  QHBoxLayout *topLeftLayout = new QHBoxLayout;
  topLeftLayout->addWidget(label);
  //topLeftLayout->addWidget(lineEdit);

  QVBoxLayout *leftLayout = new QVBoxLayout;
  leftLayout->addLayout(topLeftLayout);
  //leftLayout->addWidget(scaleCheckBox);
  leftLayout->addWidget(labScaleX);
  leftLayout->addWidget(scaleX);
  /*
  leftLayout->addWidget(labScaleY);
  leftLayout->addWidget(scaleY);
  leftLayout->addWidget(labScaleZ);
  leftLayout->addWidget(scaleZ);
  */

  //leftLayout->addWidget(traslaCheckBox);
  /*
  leftLayout->addWidget(labTraslaX);
  leftLayout->addWidget(traslaX);
  leftLayout->addWidget(labTraslaY);
  leftLayout->addWidget(traslaY);
  leftLayout->addWidget(labTraslaZ);
  leftLayout->addWidget(traslaZ);
  */

  leftLayout->addWidget(traslationGroupBox);


  //leftLayout->addWidget(rotaCheckBox);

  leftLayout->addWidget(rotationGroupBox);
  leftLayout->addWidget(frequencyGroupBox);
  //leftLayout->addWidget(labFrequency);
  //leftLayout->addWidget(frequency);
  /*
  leftLayout->addWidget(rotaX);
  leftLayout->addWidget(rotaY);
  leftLayout->addWidget(rotaZ);
  */

  leftLayout->addWidget(labOffset);
  leftLayout->addWidget(timeOffset);
  //leftLayout->addWidget(labPCAIndex);
  //leftLayout->addWidget(pcaIndex);

  leftLayout->addWidget(label2);
  leftLayout->addWidget(labelGaussNoise);
  leftLayout->addWidget(gaussianNoiseCheck);
  leftLayout->addWidget(gaussianNoiseDeviation);
  leftLayout->addWidget(labelCosmicNoise);
  leftLayout->addWidget(cosmicNoiseCheck);
  leftLayout->addWidget(cosmicNoiseDeviation);




  QHBoxLayout *buttonLayout2 = new QHBoxLayout;
  buttonLayout2->addWidget(buttonOK);
  buttonLayout2->addWidget(buttonCancel);


  QGridLayout *mainLayout = new QGridLayout;
  mainLayout->setSizeConstraint(QLayout::SetNoConstraint);
  mainLayout->addLayout(leftLayout, 0, 0);
  mainLayout->addLayout(buttonLayout2, 1,0);
  //mainLayout->addWidget(buttonBox, 0, 1);
  //mainLayout->addWidget(extension, 1, 0, 1, 2);
  mainLayout->setRowStretch(2, 1);

  setLayout(mainLayout);

  setWindowTitle(tr("sequence manager"));
  //extension->hide();
  std::cout<< "end constructor scalatraslaRota" <<std::endl;
}

void DialogScalaTraslaRota::onOK()
{
  std::cout<< "onOK" <<std::endl;
  double tx = traslaX->text().toDouble();
  double ty = traslaY->text().toDouble();
  double tz = traslaZ->text().toDouble();
  double sx = scaleX->text().toDouble();
  double sy = sx;//scaleY->text().toDouble();
  double sz = sx;//scaleZ->text().toDouble();
  double rx = rotaX->text().toDouble();
  double ry = rotaY->text().toDouble();
  double rz = rotaZ->text().toDouble();
  int gNoise;
  double gNoise_value = gaussianNoiseDeviation->text().toDouble();
  double cNoise = cosmicNoiseDeviation->text().toDouble();
  double tOffset = timeOffset->text().toDouble();
  int fType;//frequencyType
  if (freqMaxRadioButton->isChecked()){
    fType=0;
  }else {
    if (freqMinRadioButton->isChecked()){
      fType=1;
    }else if (freqCustomizedRadioButton->isChecked()){
      fType=2;
    }
  }

  if (gaussianNoiseCheck->isChecked()){
    gNoise=1;
  } else gNoise=0;
  if (cosmicNoiseCheck->isChecked()){
    cNoise=1;
  } else cNoise=0;
  double freq = frequency->text().toDouble();
  std::cout<< "onOK" <<std::endl;
  ((MainWindow*)(parent))->performModifySequence(sx,sy,sz,tx,ty,tz,rx,ry,rz,gNoise,cNoise,tOffset,fType,freq, gNoise_value);
  std::cout<< "onOK:gNoise="<<gNoise<<std::endl;
  //dialogModel = new DataDialogScalaTraslaRota(sx,sy,sz,tx,ty,tz,rx,ry,rz,gNoise,cNoise);
  dialogModel->updateData( sx,sy,sz,tx,ty,tz,rx,ry,rz,gNoise,cNoise,tOffset,fType,freq);
  std::cout<< "onOK:dialogModel->getGaussianNoiseDeviation()="<<dialogModel->getGaussianNoiseDeviation()<<std::endl;
  //((MainWindow*)(parent))->setTrasla(x,y,z);

  //((MainWindow*)(parent))->setTrasla(traslaX->text().toDouble(),traslaY->text().toDouble(),traslaZ->text().toDouble());
  //((Winslam*)(((MainWindow*)(parent))->myWinSlam))->setTrasla(traslaX,traslaY,traslaZ);
  std::cout<< "FIN onOK" <<std::endl;
  this->close();
}

void DialogScalaTraslaRota::setDataDialog(DataDialogScalaTraslaRota* aModel){
  dialogModel = aModel;
  double value = 0.0;
  traslaX->setText( QString::number(dialogModel->getTraslaX(), 'f', 6));
  traslaY->setText( QString::number(dialogModel->getTraslaY(), 'f', 6));
  traslaZ->setText( QString::number(dialogModel->getTraslaZ(), 'f', 6));
  scaleX->setText( QString::number(dialogModel->getScaleX(), 'f', 6));
  scaleY->setText( QString::number(dialogModel->getScaleX(), 'f', 6));//setText( QString::number(dialogModel->getScaleY(), 'f', 6));
  scaleZ->setText( QString::number(dialogModel->getScaleX(), 'f', 6));//setText( QString::number(dialogModel->getScaleZ(), 'f', 6));
  rotaX->setText( QString::number(dialogModel->getRotaX(), 'f', 6));
  rotaY->setText( QString::number(dialogModel->getRotaY(), 'f', 6));
  rotaZ->setText( QString::number(dialogModel->getRotaZ(), 'f', 6));
  timeOffset->setText( QString::number(dialogModel->getTimeOffset(), 'f', 6));
  gaussianNoiseDeviation->setText( QString::number(dialogModel->getGaussianNoiseDeviation(), 'f', 6));
  cosmicNoiseDeviation->setText( QString::number(dialogModel->getCosmicNoiseDeviation(), 'f', 6));

  if (dialogModel->getGaussianNoiseDeviation() == 1){
    gaussianNoiseCheck->setChecked(true);
  }

  if (dialogModel->getCosmicNoiseDeviation() == 1){
    cosmicNoiseCheck->setChecked(true);

  }
  int frequencyType=dialogModel->getFrequencyType();
  if (frequencyType==0){
    std::cout<< "DialogScalaTraslaRota::setDataDialog frequencyType="<< frequencyType <<std::endl;
    freqMaxRadioButton->setEnabled(true);
    onPressMaxFrequency();
  } else if (frequencyType==1){
    std::cout<< "DialogScalaTraslaRota::setDataDialog frequencyType="<< frequencyType <<std::endl;
    freqMinRadioButton->setEnabled(true);
    onPressMinFrequency();

  }else if (frequencyType==2){
    std::cout<< "DialogScalaTraslaRota::setDataDialog frequencyType="<< frequencyType <<std::endl;
    freqCustomizedRadioButton->setEnabled(true);
    onPressCustomizedFrequency();
  }
  frequency->setText(QString::number(dialogModel->getFrequency(),'f',6));
  //traslaX=new QString*((dialogModel->getTraslaX()));
/*
       double ty = traslaY->text().toDouble();
       double tz = traslaZ->text().toDouble();
       double sx = scaleX->text().toDouble();
       double sy = scaleY->text().toDouble();
       double sz = scaleZ->text().toDouble();
       double rx = rotaX->text().toDouble();
       double ry = rotaY->text().toDouble();
       double rz = rotaZ->text().toDouble();
  */
}


DataDialogScalaTraslaRota* DialogScalaTraslaRota::getDataDialog(){
  return dialogModel;
}

void DialogScalaTraslaRota::onCancel()
{
  std::cout<< "onCancel" <<std::endl;
  buttonCancel->hide();
  this->close();
}

void DialogScalaTraslaRota::onPressCustomizedFrequency()
{
  std::cout<<"onPressCustomizedFrequency"<<std::endl;
  frequency->setEnabled(true);
  frequency->setReadOnly(false);
  freqCustomizedRadioButton->setChecked(true);
  //frequency->setVisible(true);
  //labFrequency->setVisible(true);
  this->update();
  this->repaint();

}
void DialogScalaTraslaRota::onPressMaxFrequency()
{
  std::cout<<"onPressMaxFrequency"<<std::endl;
  frequency->setEnabled(false);
  frequency->setDisabled(true);
  freqMaxRadioButton ->setChecked(true);
  //frequency->setReadOnly(true);
  //frequency->setVisible(false);
  //labFrequency->setVisible(false);
  this->update();
  this->repaint();

}

void DialogScalaTraslaRota::onPressMinFrequency()
{
  std::cout<<"onPressMinFrequency"<<std::endl;
  frequency->setDisabled(true);
  frequency->setEnabled(false);
  freqMinRadioButton->setChecked(true);
  //frequency->setReadOnly(true);
  //frequency->setVisible(false);
  //labFrequency->setVisible(false);
  this->update();
  this->repaint();
}
