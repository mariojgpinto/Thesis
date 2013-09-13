/********************************************************************************
** Form generated from reading UI file 'PointSelectionGUI.ui'
**
** Created: Fri 13. Sep 16:56:41 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POINTSELECTIONGUI_H
#define UI_POINTSELECTIONGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PointSelectionGUI
{
public:
    QWidget *centralWidget;
    QWidget *_widget;
    QPushButton *_pushButton_exit;
    QPushButton *_pushButton_reset;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PointSelectionGUI)
    {
        if (PointSelectionGUI->objectName().isEmpty())
            PointSelectionGUI->setObjectName(QString::fromUtf8("PointSelectionGUI"));
        PointSelectionGUI->resize(660, 543);
        centralWidget = new QWidget(PointSelectionGUI);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        _widget = new QWidget(centralWidget);
        _widget->setObjectName(QString::fromUtf8("_widget"));
        _widget->setGeometry(QRect(10, 10, 640, 480));
        _pushButton_exit = new QPushButton(centralWidget);
        _pushButton_exit->setObjectName(QString::fromUtf8("_pushButton_exit"));
        _pushButton_exit->setGeometry(QRect(570, 500, 75, 23));
        _pushButton_reset = new QPushButton(centralWidget);
        _pushButton_reset->setObjectName(QString::fromUtf8("_pushButton_reset"));
        _pushButton_reset->setGeometry(QRect(480, 500, 75, 23));
        PointSelectionGUI->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(PointSelectionGUI);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        PointSelectionGUI->setStatusBar(statusBar);

        retranslateUi(PointSelectionGUI);

        QMetaObject::connectSlotsByName(PointSelectionGUI);
    } // setupUi

    void retranslateUi(QMainWindow *PointSelectionGUI)
    {
        PointSelectionGUI->setWindowTitle(QApplication::translate("PointSelectionGUI", "PointSelection", 0, QApplication::UnicodeUTF8));
        _pushButton_exit->setText(QApplication::translate("PointSelectionGUI", "Exit", 0, QApplication::UnicodeUTF8));
        _pushButton_reset->setText(QApplication::translate("PointSelectionGUI", "Reset", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PointSelectionGUI: public Ui_PointSelectionGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTSELECTIONGUI_H
