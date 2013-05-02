/********************************************************************************
** Form generated from reading UI file 'PlaneAdjustGUI.ui'
**
** Created: Mon 29. Apr 19:57:13 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLANEADJUSTGUI_H
#define UI_PLANEADJUSTGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PlaneAdjustGUI
{
public:
    QAction *_plane_action_close;
    QAction *_plane_action_save;
    QWidget *centralwidget;
    QWidget *_plane_widget;
    QPushButton *_plane_push_button_save;
    QSlider *_plane_horizontal_slider_1;
    QLabel *_plane_label_angle_1;
    QFrame *line;
    QSlider *_plane_horizontal_slider_2;
    QFrame *line_2;
    QLabel *_plane_label_angle_2;
    QPushButton *_plane_push_button_reset;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *PlaneAdjustGUI)
    {
        if (PlaneAdjustGUI->objectName().isEmpty())
            PlaneAdjustGUI->setObjectName(QString::fromUtf8("PlaneAdjustGUI"));
        PlaneAdjustGUI->resize(660, 598);
        _plane_action_close = new QAction(PlaneAdjustGUI);
        _plane_action_close->setObjectName(QString::fromUtf8("_plane_action_close"));
        _plane_action_save = new QAction(PlaneAdjustGUI);
        _plane_action_save->setObjectName(QString::fromUtf8("_plane_action_save"));
        centralwidget = new QWidget(PlaneAdjustGUI);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        _plane_widget = new QWidget(centralwidget);
        _plane_widget->setObjectName(QString::fromUtf8("_plane_widget"));
        _plane_widget->setGeometry(QRect(10, 10, 640, 480));
        _plane_push_button_save = new QPushButton(centralwidget);
        _plane_push_button_save->setObjectName(QString::fromUtf8("_plane_push_button_save"));
        _plane_push_button_save->setGeometry(QRect(570, 530, 75, 23));
        _plane_horizontal_slider_1 = new QSlider(centralwidget);
        _plane_horizontal_slider_1->setObjectName(QString::fromUtf8("_plane_horizontal_slider_1"));
        _plane_horizontal_slider_1->setGeometry(QRect(10, 520, 251, 19));
        _plane_horizontal_slider_1->setOrientation(Qt::Horizontal);
        _plane_label_angle_1 = new QLabel(centralwidget);
        _plane_label_angle_1->setObjectName(QString::fromUtf8("_plane_label_angle_1"));
        _plane_label_angle_1->setGeometry(QRect(10, 500, 46, 13));
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(270, 500, 20, 61));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        _plane_horizontal_slider_2 = new QSlider(centralwidget);
        _plane_horizontal_slider_2->setObjectName(QString::fromUtf8("_plane_horizontal_slider_2"));
        _plane_horizontal_slider_2->setGeometry(QRect(290, 520, 251, 19));
        _plane_horizontal_slider_2->setOrientation(Qt::Horizontal);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(550, 500, 20, 61));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);
        _plane_label_angle_2 = new QLabel(centralwidget);
        _plane_label_angle_2->setObjectName(QString::fromUtf8("_plane_label_angle_2"));
        _plane_label_angle_2->setGeometry(QRect(290, 500, 46, 13));
        _plane_push_button_reset = new QPushButton(centralwidget);
        _plane_push_button_reset->setObjectName(QString::fromUtf8("_plane_push_button_reset"));
        _plane_push_button_reset->setGeometry(QRect(570, 500, 75, 23));
        PlaneAdjustGUI->setCentralWidget(centralwidget);
        menubar = new QMenuBar(PlaneAdjustGUI);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 660, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        PlaneAdjustGUI->setMenuBar(menubar);
        statusbar = new QStatusBar(PlaneAdjustGUI);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        PlaneAdjustGUI->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(_plane_action_save);
        menuFile->addSeparator();
        menuFile->addAction(_plane_action_close);

        retranslateUi(PlaneAdjustGUI);

        QMetaObject::connectSlotsByName(PlaneAdjustGUI);
    } // setupUi

    void retranslateUi(QMainWindow *PlaneAdjustGUI)
    {
        PlaneAdjustGUI->setWindowTitle(QApplication::translate("PlaneAdjustGUI", "MainWindow", 0, QApplication::UnicodeUTF8));
        _plane_action_close->setText(QApplication::translate("PlaneAdjustGUI", "Close", 0, QApplication::UnicodeUTF8));
        _plane_action_save->setText(QApplication::translate("PlaneAdjustGUI", "Save", 0, QApplication::UnicodeUTF8));
        _plane_push_button_save->setText(QApplication::translate("PlaneAdjustGUI", "Save", 0, QApplication::UnicodeUTF8));
        _plane_label_angle_1->setText(QApplication::translate("PlaneAdjustGUI", "Angle1", 0, QApplication::UnicodeUTF8));
        _plane_label_angle_2->setText(QApplication::translate("PlaneAdjustGUI", "Angle2", 0, QApplication::UnicodeUTF8));
        _plane_push_button_reset->setText(QApplication::translate("PlaneAdjustGUI", "Reset", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("PlaneAdjustGUI", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PlaneAdjustGUI: public Ui_PlaneAdjustGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLANEADJUSTGUI_H
