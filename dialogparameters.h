#ifndef DIALOGPARAMETERS_H
#define DIALOGPARAMETERS_H
#include <QDialog>
#include "datadialogparameters.h"

class QCheckBox;
class QDialogButtonBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextBox;
class MainWindow;
class QRadioButton;

class DialogParameters: public QDialog
{
    Q_OBJECT

public:
    DialogParameters();
    DialogParameters(QWidget *parent = 0);
    DataDialogParameters* dataModel;
    MainWindow *parent;
private:
    QLabel *labMaxLines;
    QLineEdit *MaxLines;
    QLabel *labWindowOffset;
    QLineEdit *windowOffset;
    QLabel *labStepOffset;
    QLineEdit *stepOffset;
    QLabel *labMessageCLEAN;
    QPushButton *buttonOK;
    QPushButton *buttonCancel;

public slots:
    void onOK();
    void onCancel();


public:
   void setDataDialog(DataDialogParameters* aModel);


};
#endif // DIALOGPARAMETERS_H
