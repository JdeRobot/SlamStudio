
#include "dialogshowestimated.h"


DialogShowEstimated::DialogShowEstimated(QWidget *parent)
    : QDialog(parent)
{
    this->parent=(MainWindow *)parent;
    std::cout<< "constructor DialogShowEstimated" <<std::endl;
    labelTitle = new QLabel(tr("Show ESTIMATED transformations:"));


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
    labScaleZ = new QLabel(tr("scale Z"));
    scaleZ = new QLineEdit;
    scaleZ->setText("0");
    scaleZ->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    scaleZ->setMaximumWidth(100);

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
    labTimeOffset = new QLabel(tr("Time Offset"));
    timeOffset = new QLineEdit;
    timeOffset->setText("0");
    timeOffset->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    timeOffset->setMaximumWidth(100);

    labRMax = new QLabel(tr("Correlation Max value"));
    rMax = new QLineEdit;
    rMax->setText("0");
    rMax->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    rMax->setMaximumWidth(100);

    labRotationMatrix = new QLabel(tr("Rotation Matrix Estimated"));
    rotationMatrix = new QTextEdit;
    rotationMatrix->setText("0 0 0 \n 0 0 0 \n 0 0 0\n");
    rotationMatrix->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    rotationMatrix->setMaximumHeight(130);
    rotationMatrix->setMaximumWidth(230);

    labYaw = new QLabel(tr("Yaw"));
    yaw = new QLineEdit;
    yaw->setText("0");
    yaw->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    yaw->setMaximumWidth(100);
    labPitch = new QLabel(tr("Pitch"));
    pitch = new QLineEdit;
    pitch->setText("0");
    pitch->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    pitch->setMaximumWidth(100);
    labRoll = new QLabel(tr("Roll"));
    roll = new QLineEdit;
    roll->setText("0");
    roll->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    roll->setMaximumWidth(100);

    labRMSE = new QLabel(tr("RMSE"));
    RMSE = new QLineEdit;
    RMSE->setText("0");
    RMSE->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    RMSE->setMaximumWidth(100);


    buttonOK = new QPushButton(tr("&OK"));

    buttonCancel = new QPushButton(tr("&Cancel"));

    connect(buttonOK, SIGNAL(clicked()), this, SLOT(onOK()));
    connect(buttonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));

    std::cout<< "2 constructor  DialogShowEstimated" <<std::endl;


    QHBoxLayout *topLeftLayout = new QHBoxLayout;
    topLeftLayout->addWidget(labelTitle);
    //topLeftLayout->addWidget(lineEdit);

    QVBoxLayout *leftLayout = new QVBoxLayout;
    leftLayout->addLayout(topLeftLayout);

    leftLayout->addWidget(labScaleX);
    leftLayout->addWidget(scaleX);
    leftLayout->addWidget(labScaleY);
    leftLayout->addWidget(scaleY);
    leftLayout->addWidget(labScaleZ);
    leftLayout->addWidget(scaleZ);

    leftLayout->addWidget(labTraslaX);
    leftLayout->addWidget(traslaX);
    leftLayout->addWidget(labTraslaY);
    leftLayout->addWidget(traslaY);
    leftLayout->addWidget(labTraslaZ);
    leftLayout->addWidget(traslaZ);
    leftLayout->addWidget(labTimeOffset);
    leftLayout->addWidget(timeOffset);
    leftLayout->addWidget(labRMax);
    leftLayout->addWidget(rMax);
    leftLayout->addWidget(labYaw);
    leftLayout->addWidget(yaw);
    leftLayout->addWidget(labPitch);
    leftLayout->addWidget(pitch);
    leftLayout->addWidget(labRoll);
    leftLayout->addWidget(roll);

    leftLayout->addWidget(labRMSE);
    leftLayout->addWidget(RMSE);


    QHBoxLayout *buttonLayout2 = new QHBoxLayout;
    buttonLayout2->addWidget(buttonOK);
    buttonLayout2->addWidget(buttonCancel);


    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);
    mainLayout->addLayout(leftLayout, 0, 0);
    mainLayout->addLayout(buttonLayout2, 1,0);
    mainLayout->setRowStretch(2, 1);

    setLayout(mainLayout);

    setWindowTitle(tr("Show Estimated transformations"));
    //extension->hide();
    std::cout<< "FIN constructor DialogShowEstimated" <<std::endl;
}

void DialogShowEstimated::onOK()
{
   std::cout<< "FIN onOK" <<std::endl;
   this->close();
}

void DialogShowEstimated::setDataDialog(DataDialogShowEstimated* aModel){
    dialogModel = aModel;
    double value = 0.0;
    std::cout<< "DialogShowEstimated::setDataDialog " <<  dialogModel->getDialogType() <<std::endl;
    if (dialogModel->getDialogType() == 0){

        labelTitle->setText("Show ESTIMATED transformations A TO B:");

    } else if (dialogModel->getDialogType()==1) {


        labelTitle->setText("Show ESTIMATED transformations B TO A:");
    } else if (dialogModel->getDialogType()==2) {


        labelTitle->setText("Show ESTIMATED transformations A TO B RANSAC:");
    } else if (dialogModel->getDialogType()==3) {


        labelTitle->setText("Show ESTIMATED transformations B TO A RANSAC:");
    }
   double myScale=dialogModel->getScaleX();
   traslaX->setText( QString::number((dialogModel->getTraslaX()*myScale), 'f', 6));
   traslaY->setText( QString::number((dialogModel->getTraslaY()*myScale), 'f', 6));
   traslaZ->setText( QString::number((dialogModel->getTraslaZ()*myScale), 'f', 6));
   scaleX->setText( QString::number(dialogModel->getScaleX(), 'f', 6));
   scaleY->setText( QString::number(dialogModel->getScaleY(), 'f', 6));
   scaleZ->setText( QString::number(dialogModel->getScaleZ(), 'f', 6));
   timeOffset->setText( QString::number(dialogModel->getTimeOffset(), 'f', 6));
   rMax->setText( QString::number(dialogModel->getRMax(), 'f', 6));
   yaw->setText(QString::number(dialogModel->getYaw(),'f',6));
   pitch->setText(QString::number(dialogModel->getPitch(),'f',6));
   roll->setText(QString::number(dialogModel->getRoll(),'f',6));

   RMSE->setText(QString::number(dialogModel->getRMSE(),'f',6));

}


DataDialogShowEstimated* DialogShowEstimated::getDataDialog(){
    return dialogModel;
}

void DialogShowEstimated::onCancel()
{
   std::cout<< "onCancel" <<std::endl;
   buttonCancel->hide();
   this->close();
}

