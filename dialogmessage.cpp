#include <QtWidgets>
#include <iostream>
#include "dialogmessage.h"
#include "mainwindow.h"



DialogMessage::DialogMessage(QWidget *parent)
    : QDialog(parent)
{
    this->parent=(MainWindow *)parent;
    std::cout<< "constructor DialogMessage" <<std::endl;
    labelTitle = new QLabel(tr("Information"));
    //lineEdit = new QLineEdit;
    //label->setBuddy(lineEdit);


    labelMessage = new QLabel(tr("Please indicate transformation parameters ..."));
    //buttonOK = new QPushButton(tr("&Close"));

    //buttonCancel = new QPushButton(tr("&Cancel"));

    //connect(buttonOK, SIGNAL(clicked()), this, SLOT(onOK()));
    //connect(buttonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));


    QHBoxLayout *topLeftLayout = new QHBoxLayout;
    topLeftLayout->addWidget(labelTitle);
    //topLeftLayout->addWidget(lineEdit);

    QVBoxLayout *leftLayout = new QVBoxLayout;
    leftLayout->addLayout(topLeftLayout);
    leftLayout->addWidget(labelMessage);

    //QHBoxLayout *buttonLayout2 = new QHBoxLayout;
    //buttonLayout2->addWidget(buttonOK);
    //buttonLayout2->addWidget(buttonCancel);

    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);

    mainLayout->addLayout(leftLayout, 0, 0);
    //mainLayout->addLayout(buttonLayout2,0,1);

    //mainLayout->addWidget(buttonBox, 0, 1);
    //mainLayout->addWidget(extension, 1, 0, 1, 2);
    mainLayout->setRowStretch(2, 1);

    setLayout(mainLayout);

    setWindowTitle(tr(" Information "));

    //extension->hide();
    std::cout<< "FIN constructor DialogMessage" <<std::endl;
}

void DialogMessage::setText(QString aText){
    labelMessage->setText(aText);
    this->update();
    this->repaint();
}

void DialogMessage::onOK()
{
   std::cout<< "FIN onOK" <<std::endl;
   this->close();
}

void DialogMessage::onCancel()
{
   std::cout<< "onCancel" <<std::endl;
   buttonCancel->hide();
   this->close();
}




