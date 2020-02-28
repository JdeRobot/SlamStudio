#ifndef DIALOGMESSAGE_H
#define DIALOGMESSAGE_H
#include <QDialog>
#include <QtWidgets>
#include <iostream>

class QDialogButtonBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;
class QTextBox;
class MainWindow;

class DialogMessage : public QDialog
{
    Q_OBJECT

public:
    DialogMessage(QWidget *parent = 0);
    //DialogScalaTraslaRota(QWidget *parent = 0, DataDialogScalaTraslaRota* aModel );
    MainWindow *parent;

private:
    QLabel *labelTitle;
    QLabel *labelMessage;
    QPushButton *buttonOK;
    QPushButton *buttonCancel;

public slots:
    void onOK();
    void onCancel();

};

#endif // DIALOGMESSAGE_H
