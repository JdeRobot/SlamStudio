#-------------------------------------------------
#
# Project created by QtCreator 2018-08-25T00:22:41
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = helloQT4
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    glwidget.cpp \
    logo.cpp \
    window.cpp \
    tetrahedron.cpp \
    winslam.cpp \
    dialogscalatraslarota.cpp \
    Point3D.cpp \
    transformador2/Transformador.cpp \
    Registrador/Registrador.cpp \
    GeneratorPCA/GeneratorPCA.cpp \
    ModuloEscala/FindScala.cpp \
    AjusteTiempo/AjusteTiempo.cpp \
    Interpolator/Interpolator.cpp \
    datadialogscalatraslarota.cpp \
    dialogshowestimated.cpp \
    datadialogshowestimated.cpp

HEADERS += \
        mainwindow.h \
    glwidget.h \
    logo.h \
    window.h \
    winslam.h \
    tetrahedron.h \
    dialogscalatraslarota.h \
    transformador2\transformador.h \
    transformador2\Point3D.h \
    Point3D.h \
    transformador2/Transformador.h \
    Registrador/Registrador.h \
    GeneratorPCA/GeneratorPCA.h \
    ModuloEscala/FindScala.h \
    AjusteTiempo/AjusteTiempo.h \
    Interpolator/Interpolator.h \
    datadialogscalatraslarota.h \
    dialogshowestimated.h \
    datadialogshowestimated.h

FORMS += \
        mainwindow.ui
INCLUDEPATH += ..\transformador2


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
