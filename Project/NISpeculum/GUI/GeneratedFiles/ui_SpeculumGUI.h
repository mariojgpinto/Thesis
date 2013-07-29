/********************************************************************************
** Form generated from reading UI file 'SpeculumGUI.ui'
**
** Created: Sun 28. Jul 02:07:59 2013
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
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SpeculumGUI
{
public:
    QAction *_action_close;
    QWidget *centralWidget;
    QWidget *_widget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *SpeculumGUI)
    {
        if (SpeculumGUI->objectName().isEmpty())
            SpeculumGUI->setObjectName(QString::fromUtf8("SpeculumGUI"));
        SpeculumGUI->resize(660, 535);
        _action_close = new QAction(SpeculumGUI);
        _action_close->setObjectName(QString::fromUtf8("_action_close"));
        centralWidget = new QWidget(SpeculumGUI);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        _widget = new QWidget(centralWidget);
        _widget->setObjectName(QString::fromUtf8("_widget"));
        _widget->setGeometry(QRect(10, 10, 640, 480));
        SpeculumGUI->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(SpeculumGUI);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 660, 21));
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
        menuFile->addAction(_action_close);

        retranslateUi(SpeculumGUI);

        QMetaObject::connectSlotsByName(SpeculumGUI);
    } // setupUi

    void retranslateUi(QMainWindow *SpeculumGUI)
    {
        SpeculumGUI->setWindowTitle(QApplication::translate("SpeculumGUI", "SpeculumGUI", 0, QApplication::UnicodeUTF8));
        _action_close->setText(QApplication::translate("SpeculumGUI", "Close", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("SpeculumGUI", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SpeculumGUI: public Ui_SpeculumGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SPECULUMGUI_H
