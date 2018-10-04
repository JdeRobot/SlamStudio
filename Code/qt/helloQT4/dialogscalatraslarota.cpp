#include <QtWidgets>
#include <iostream>
#include "dialogscalatraslarota.h"
#include "mainwindow.h"

DialogScalaTraslaRota::DialogScalaTraslaRota(QWidget *parent)
    : QDialog(parent)
{
    this->parent=(MainWindow *)parent;
    std::cout<< "constructor DialogScalaTraslaRota" <<std::endl;
    label = new QLabel(tr("Configure Modifications on data sequence:"));
    //lineEdit = new QLineEdit;
    //label->setBuddy(lineEdit);

    scaleCheckBox = new QCheckBox(tr("Scale"));
    labScaleX = new QLabel(tr("scale X"));
    scaleX = new QLineEdit;
    scaleX->setText("0");
    scaleX->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    scaleX->setMaximumWidth(100);
    labScaleY = new QLabel(tr("scale Y"));
    scaleY = new QLineEdit;
    scaleY->setText("0");
    scaleY->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    scaleY->setMaximumWidth(100);
    labScaleZ = new QLabel(tr("scale X"));
    scaleZ = new QLineEdit;
    scaleZ->setText("0");
    scaleZ->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    scaleZ->setMaximumWidth(100);


    traslaCheckBox = new QCheckBox(tr("trasla"));
    labTraslaX = new QLabel(tr("trasla X"));
    traslaX = new QLineEdit;
    traslaX->setText("0");
    traslaX->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    traslaX->setMaximumWidth(100);
    labTraslaY = new QLabel(tr("trasla Y"));
    traslaY = new QLineEdit;
    traslaY->setText("0");
    traslaY->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    traslaY->setMaximumWidth(100);
    labTraslaZ = new QLabel(tr("trasla Z"));
    traslaZ = new QLineEdit;
    traslaZ->setText("0");
    traslaZ->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    traslaZ->setMaximumWidth(100);
    labOffset = new QLabel(tr("Time Offset"));
    timeOffset = new QLineEdit;
    timeOffset->setText("0");
    timeOffset->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    timeOffset->setMaximumWidth(100);
    /*traslaCheckBox = new QCheckBox(tr("Traslation"));
    traslaX = new QLineEdit;
    traslaX->setText("0");
    traslaY = new QLineEdit;
    traslaY->setText("0");
    traslaZ = new QLineEdit;
    traslaZ->setText("0");
    */
    QGroupBox *rotationGroupBox = new QGroupBox(tr("ROTATION"));
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

    buttonOK = new QPushButton(tr("&OK"));

    buttonCancel = new QPushButton(tr("&Cancel"));

    connect(buttonOK, SIGNAL(clicked()), this, SLOT(onOK()));
    connect(buttonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));

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

    QHBoxLayout *topLeftLayout = new QHBoxLayout;
    topLeftLayout->addWidget(label);
    //topLeftLayout->addWidget(lineEdit);

    QVBoxLayout *leftLayout = new QVBoxLayout;
    leftLayout->addLayout(topLeftLayout);
    leftLayout->addWidget(scaleCheckBox);
    leftLayout->addWidget(labScaleX);
    leftLayout->addWidget(scaleX);
    leftLayout->addWidget(labScaleY);
    leftLayout->addWidget(scaleY);
    leftLayout->addWidget(labScaleZ);
    leftLayout->addWidget(scaleZ);
    //leftLayout->addWidget(traslaCheckBox);
    leftLayout->addWidget(labTraslaX);
    leftLayout->addWidget(traslaX);
    leftLayout->addWidget(labTraslaY);
    leftLayout->addWidget(traslaY);
    leftLayout->addWidget(labTraslaZ);
    leftLayout->addWidget(traslaZ);
    leftLayout->addWidget(labOffset);
    leftLayout->addWidget(timeOffset);
    //leftLayout->addWidget(rotaCheckBox);
    leftLayout->addWidget(rotationGroupBox);
    /*
    leftLayout->addWidget(rotaX);
    leftLayout->addWidget(rotaY);
    leftLayout->addWidget(rotaZ);
    */

    leftLayout->addWidget(label2);
    leftLayout->addWidget(labelGaussNoise);

    leftLayout->addWidget(gaussianNoiseDeviation);
    leftLayout->addWidget(labelCosmicNoise);
    leftLayout->addWidget(cosmicNoiseDeviation);



    QHBoxLayout *buttonLayout2 = new QHBoxLayout;
    buttonLayout2->addWidget(buttonOK);
    buttonLayout2->addWidget(buttonCancel);


    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);
    mainLayout->addLayout(leftLayout, 0, 0);
    mainLayout->addLayout(buttonLayout2, 1,0);
    //mainLayout->addWidget(buttonBox, 0, 1);
    //mainLayout->addWidget(extension, 1, 0, 1, 2);
    mainLayout->setRowStretch(2, 1);

    setLayout(mainLayout);

    setWindowTitle(tr("sequence manager"));
    //extension->hide();
}

void DialogScalaTraslaRota::onOK()
{
   std::cout<< "onOK" <<std::endl;
   double tx = traslaX->text().toDouble();
   double ty = traslaY->text().toDouble();
   double tz = traslaZ->text().toDouble();
   double sx = scaleX->text().toDouble();
   double sy = scaleY->text().toDouble();
   double sz = scaleZ->text().toDouble();
   double rx = rotaX->text().toDouble();
   double ry = rotaY->text().toDouble();
   double rz = rotaZ->text().toDouble();
   double gNoise = gaussianNoiseDeviation->text().toDouble();
   double cNoise = cosmicNoiseDeviation->text().toDouble();
   double tOffset = timeOffset->text().toDouble();
   std::cout<< "onOK" <<std::endl;
   ((MainWindow*)(parent))->performModifySequence(sx,sy,sz,tx,ty,tz,rx,ry,rz,gNoise,cNoise,tOffset);
   std::cout<< "onOK:gNoise="<<gNoise<<std::endl;
   //dialogModel = new DataDialogScalaTraslaRota(sx,sy,sz,tx,ty,tz,rx,ry,rz,gNoise,cNoise);
   dialogModel->updateData( sx,sy,sz,tx,ty,tz,rx,ry,rz,gNoise,cNoise,tOffset);
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
   scaleY->setText( QString::number(dialogModel->getScaleY(), 'f', 6));
   scaleZ->setText( QString::number(dialogModel->getScaleZ(), 'f', 6));
   rotaX->setText( QString::number(dialogModel->getRotaX(), 'f', 6));
   rotaY->setText( QString::number(dialogModel->getRotaY(), 'f', 6));
   rotaZ->setText( QString::number(dialogModel->getRotaZ(), 'f', 6));
   timeOffset->setText( QString::number(dialogModel->getTimeOffset(), 'f', 6));
   gaussianNoiseDeviation->setText( QString::number(dialogModel->getGaussianNoiseDeviation(), 'f', 6));
   cosmicNoiseDeviation->setText( QString::number(dialogModel->getCosmicNoiseDeviation(), 'f', 6));
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
