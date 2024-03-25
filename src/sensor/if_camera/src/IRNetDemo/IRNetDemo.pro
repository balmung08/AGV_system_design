#-------------------------------------------------
#
# Project created by QtCreator 2021-03-11T00:03:29
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = IRNetDemo
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    playerwidget.cpp \
    analysisgraphicswidget.cpp \
    analysisgraphicsitem.cpp \
    irnetsdkmanager.cpp \
    superslider.cpp \
    utils/config.cpp

HEADERS  += mainwindow.h \
    playerwidget.h \
    common.h \
    analysisgraphicswidget.h \
    analysisgraphicsitem.h \
    irnetsdkmanager.h \
    superslider.h \
    utils/config.h


FORMS    += mainwindow.ui

UI_DIR=./UI


#unix:!macx: LIBS += -L$$PWD/../lib -lIRNet -lrtspclient -lopencv_world

#INCLUDEPATH += $$PWD/../include
#DEPENDPATH += $$PWD/../include


msvc {
    QMAKE_CFLAGS += /utf-8
    QMAKE_CXXFLAGS += /utf-8

}




DESTDIR += $$PWD/../../tool/


LIBS += -L$$PWD/../../lib/ -lIRNet

INCLUDEPATH += $$PWD/../../include
DEPENDPATH += $$PWD/../../include

