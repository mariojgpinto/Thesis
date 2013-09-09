/********************************************************************************
** Form generated from reading UI file 'SpeculumGUI.ui'
**
** Created: Sat 7. Sep 17:45:51 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SPECULUMGUI_H
#define UI_SPECULUMGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SpeculumGUI
{
public:
    QAction *_action_close;
    QAction *_action_save;
    QAction *_action_load;
    QWidget *centralWidget;
    QWidget *_widget;
    QPushButton *_main_pushButton_add_floor;
    QPushButton *_main_pushButton_add_mirror;
    QPushButton *_main_pushButton_pause;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *SpeculumGUI)
    {
        if (SpeculumGUI->objectName().isEmpty())
            SpeculumGUI->setObjectName(QString::fromUtf8("SpeculumGUI"));
        SpeculumGUI->resize(660, 581);
        _action_close = new QAction(SpeculumGUI);
        _action_close->setObjectName(QString::fromUtf8("_action_close"));
        _action_save = new QAction(SpeculumGUI);
        _action_save->setObjectName(QString::fromUtf8("_action_save"));
        _action_load = new QAction(SpeculumGUI);
        _action_load->setObjectName(QString::fromUtf8("_action_load"));
        centralWidget = new QWidget(SpeculumGUI);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        _widget = new QWidget(centralWidget);
        _widget->setObjectName(QString::fromUtf8("_widget"));
        _widget->setGeometry(QRect(10, 40, 640, 480));
        _main_pushButton_add_floor = new QPushButton(centralWidget);
        _main_pushButton_add_floor->setObjectName(QString::fromUtf8("_main_pushButton_add_floor"));
        _main_pushButton_add_floor->setGeometry(QRect(10, 10, 75, 23));
        _main_pushButton_add_mirror = new QPushButton(centralWidget);
        _main_pushButton_add_mirror->setObjectName(QString::fromUtf8("_main_pushButton_add_mirror"));
        _main_pushButton_add_mirror->setGeometry(QRect(90, 10, 75, 23));
        _main_pushButton_pause = new QPushButton(centralWidget);
        _main_pushButton_pause->setObjectName(QString::fromUtf8("_main_pushButton_pause"));
        _main_pushButton_pause->setGeometry(QRect(570, 10, 75, 23));
        SpeculumGUI->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(SpeculumGUI);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 660, 18));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        SpeculumGUI->setMenuBar(menuBar);
        mainToolBar = new QToolBar(SpeculumGUI);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        SpeculumGUI->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(SpeculumGUI);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        SpeculumGUI->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addAction(_action_load);
        menuFile->addAction(_action_save);
        menuFile->addSeparator();
        menuFile->addAction(_action_close);

        retranslateUi(SpeculumGUI);

        QMetaObject::connectSlotsByName(SpeculumGUI);
    } // setupUi

    void retranslateUi(QMainWindow *SpeculumGUI)
    {
        SpeculumGUI->setWindowTitle(QApplication::translate("SpeculumGUI", "SpeculumGUI", 0, QApplication::UnicodeUTF8));
        _action_close->setText(QApplication::translate("SpeculumGUI", "Close", 0, QApplication::UnicodeUTF8));
        _action_save->setText(QApplication::translate("SpeculumGUI", "Save", 0, QApplication::UnicodeUTF8));
        _action_load->setText(QApplication::translate("SpeculumGUI", "Load", 0, QApplication::UnicodeUTF8));
        _main_pushButton_add_floor->setText(QApplication::translate("SpeculumGUI", "Add Floor", 0, QApplication::UnicodeUTF8));
        _main_pushButton_add_mirror->setText(QApplication::translate("SpeculumGUI", "Add Mirror", 0, QApplication::UnicodeUTF8));
        _main_pushButton_pause->setText(QApplication::translate("SpeculumGUI", "Pause", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("SpeculumGUI", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SpeculumGUI: public Ui_SpeculumGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SPECULUMGUI_H
