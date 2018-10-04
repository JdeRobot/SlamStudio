#include <QtWidgets>
#include <iostream>
#include "dialogshowestimated.h"
#include "mainwindow.h"

DialogShowEstimated::DialogShowEstimated(QWidget *parent)
    : QDialog(parent)
{
    this->parent=(MainWindow *)parent;
    std::cout<< "constructor DialogShowEstimated" <<std::endl;
    label = new QLabel(tr("Show ESTIMATED transformations:"));
    //lineEdit = new QLineEdit;
    //label->setBuddy(lineEdit);


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





    buttonOK = new QPushButton(tr("&OK"));

    buttonCancel = new QPushButton(tr("&Cancel"));

    connect(buttonOK, SIGNAL(clicked()), this, SLOT(onOK()));
    connect(buttonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));



    QHBoxLayout *topLeftLayout = new QHBoxLayout;
    topLeftLayout->addWidget(label);
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
    leftLayout->addWidget(labRotationMatrix);
    leftLayout->addWidget(rotationMatrix);



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

    setWindowTitle(tr("Show Estimated transformations"));
    //extension->hide();
}

void DialogShowEstimated::onOK()
{
   std::cout<< "FIN onOK" <<std::endl;
   this->close();
}

void DialogShowEstimated::setDataDialog(DataDialogShowEstimated* aModel){
    dialogModel = aModel;
    double value = 0.0;
   traslaX->setText( QString::number(dialogModel->getTraslaX(), 'f', 6));
   traslaY->setText( QString::number(dialogModel->getTraslaY(), 'f', 6));
   traslaZ->setText( QString::number(dialogModel->getTraslaZ(), 'f', 6));
   scaleX->setText( QString::number(dialogModel->getScaleX(), 'f', 6));
   scaleY->setText( QString::number(dialogModel->getScaleY(), 'f', 6));
   scaleZ->setText( QString::number(dialogModel->getScaleZ(), 'f', 6));
   timeOffset->setText( QString::number(dialogModel->getTimeOffset(), 'f', 6));
   rMax->setText( QString::number(dialogModel->getRMax(), 'f', 6));
   rotationMatrix->setText(QString::number(dialogModel->getRotaX1(), 'f', 6)+"  "+QString::number(dialogModel->getRotaY1(), 'f', 6)+"  "+QString::number(dialogModel->getRotaZ1(), 'f', 6)+"\n\n"+
                           QString::number(dialogModel->getRotaX2(), 'f', 6)+"  "+QString::number(dialogModel->getRotaY2(), 'f', 6)+"  "+QString::number(dialogModel->getRotaZ2(), 'f', 6)+"\n\n"+
                           QString::number(dialogModel->getRotaX3(), 'f', 6)+"  "+QString::number(dialogModel->getRotaY3(), 'f', 6)+"  "+QString::number(dialogModel->getRotaZ3(), 'f', 6));

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

