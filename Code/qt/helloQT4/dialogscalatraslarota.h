#ifndef DIALOGSCALATRASLAROTA_H
#define DIALOGSCALATRASLAROTA_H
#include <QDialog>
//#include "mainwindow.h"

class QCheckBox;
class QDialogButtonBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextBox;
class MainWindow;

class DialogScalaTraslaRota : public QDialog
{
    Q_OBJECT

public:
    DialogScalaTraslaRota(QWidget *parent = 0);
    MainWindow *parent;

private:
    QLabel *label;
    QLineEdit *saveFile;
    QLineEdit *scaleX;
    QLineEdit *scaleY;
    QLineEdit *scaleZ;
    QLineEdit *traslaX;
    QLineEdit *traslaY;
    QLineEdit *traslaZ;
    QLineEdit *rotaX;
    QLineEdit *rotaY;
    QLineEdit *rotaZ;
    QCheckBox *scaleCheckBox;
    QCheckBox *traslaCheckBox;
    QCheckBox *rotaCheckBox;
    QDialogButtonBox *buttonBox;
    QPushButton *modifyButton;
    //QPushButton *moreButton;
    QWidget *extension;
    QPushButton *buttonOK;
    QPushButton *buttonCancel;
public slots:
    void onOK();
    void onCancel();
};

#endif // DIALOGSCALATRASLAROTA_H
