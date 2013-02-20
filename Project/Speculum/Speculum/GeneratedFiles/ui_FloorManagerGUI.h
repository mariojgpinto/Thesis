/********************************************************************************
** Form generated from reading UI file 'FloorManagerGUI.ui'
**
** Created: Tue 19. Feb 22:53:33 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FLOORMANAGERGUI_H
#define UI_FLOORMANAGERGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FloorManagerGUI
{
public:
    QAction *_floor_action_close;
    QWidget *centralwidget;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *FloorManagerGUI)
    {
        if (FloorManagerGUI->objectName().isEmpty())
            FloorManagerGUI->setObjectName(QString::fromUtf8("FloorManagerGUI"));
        FloorManagerGUI->resize(800, 600);
        _floor_action_close = new QAction(FloorManagerGUI);
        _floor_action_close->setObjectName(QString::fromUtf8("_floor_action_close"));
        centralwidget = new QWidget(FloorManagerGUI);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        FloorManagerGUI->setCentralWidget(centralwidget);
        menubar = new QMenuBar(FloorManagerGUI);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        FloorManagerGUI->setMenuBar(menubar);
        statusbar = new QStatusBar(FloorManagerGUI);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        FloorManagerGUI->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(_floor_action_close);

        retranslateUi(FloorManagerGUI);

        QMetaObject::connectSlotsByName(FloorManagerGUI);
    } // setupUi

    void retranslateUi(QMainWindow *FloorManagerGUI)
    {
        FloorManagerGUI->setWindowTitle(QApplication::translate("FloorManagerGUI", "MainWindow", 0, QApplication::UnicodeUTF8));
        _floor_action_close->setText(QApplication::translate("FloorManagerGUI", "Close", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("FloorManagerGUI", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class FloorManagerGUI: public Ui_FloorManagerGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FLOORMANAGERGUI_H
