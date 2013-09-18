/********************************************************************************
** Form generated from reading UI file 'FloorManagerGUI.ui'
**
** Created: Mon 16. Sep 22:44:08 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FLOORMANAGERGUI_H
#define UI_FLOORMANAGERGUI_H

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
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FloorManager
{
public:
    QAction *_floor_action_close;
    QWidget *centralwidget;
    QLabel *_floor_label_a;
    QSlider *_floor_horizontal_slider_a;
    QLabel *_floor_label_b;
    QSlider *_floor_horizontal_slider_b;
    QLabel *_floor_label_c;
    QSlider *_floor_horizontal_slider_c;
    QLabel *_floor_label_d;
    QSlider *_floor_horizontal_slider_d;
    QWidget *_floor_widget;
    QFrame *line;
    QFrame *line_2;
    QFrame *line_3;
    QSlider *_floor_horizontal_slider_max;
    QSlider *_floor_horizontal_slider_min;
    QFrame *line_4;
    QLabel *_floor_label_min;
    QLabel *_floor_label_max;
    QFrame *line_5;
    QPushButton *_floor_push_button_add;
    QLabel *_floor_label_thresh;
    QSlider *_floor_horizontal_slider_thresh;
    QMenuBar *menubar;
    QMenu *menuFile;

    void setupUi(QMainWindow *FloorManager)
    {
        if (FloorManager->objectName().isEmpty())
            FloorManager->setObjectName(QString::fromUtf8("FloorManager"));
        FloorManager->resize(540, 496);
        _floor_action_close = new QAction(FloorManager);
        _floor_action_close->setObjectName(QString::fromUtf8("_floor_action_close"));
        centralwidget = new QWidget(FloorManager);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        _floor_label_a = new QLabel(centralwidget);
        _floor_label_a->setObjectName(QString::fromUtf8("_floor_label_a"));
        _floor_label_a->setGeometry(QRect(10, 360, 61, 16));
        _floor_horizontal_slider_a = new QSlider(centralwidget);
        _floor_horizontal_slider_a->setObjectName(QString::fromUtf8("_floor_horizontal_slider_a"));
        _floor_horizontal_slider_a->setGeometry(QRect(100, 360, 431, 20));
        _floor_horizontal_slider_a->setMinimum(-1000);
        _floor_horizontal_slider_a->setMaximum(1000);
        _floor_horizontal_slider_a->setOrientation(Qt::Horizontal);
        _floor_label_b = new QLabel(centralwidget);
        _floor_label_b->setObjectName(QString::fromUtf8("_floor_label_b"));
        _floor_label_b->setGeometry(QRect(10, 390, 61, 16));
        _floor_horizontal_slider_b = new QSlider(centralwidget);
        _floor_horizontal_slider_b->setObjectName(QString::fromUtf8("_floor_horizontal_slider_b"));
        _floor_horizontal_slider_b->setGeometry(QRect(100, 390, 431, 20));
        _floor_horizontal_slider_b->setMinimum(-1000);
        _floor_horizontal_slider_b->setMaximum(1000);
        _floor_horizontal_slider_b->setOrientation(Qt::Horizontal);
        _floor_label_c = new QLabel(centralwidget);
        _floor_label_c->setObjectName(QString::fromUtf8("_floor_label_c"));
        _floor_label_c->setGeometry(QRect(10, 420, 61, 16));
        _floor_horizontal_slider_c = new QSlider(centralwidget);
        _floor_horizontal_slider_c->setObjectName(QString::fromUtf8("_floor_horizontal_slider_c"));
        _floor_horizontal_slider_c->setGeometry(QRect(100, 420, 431, 20));
        _floor_horizontal_slider_c->setMinimum(-1000);
        _floor_horizontal_slider_c->setMaximum(1000);
        _floor_horizontal_slider_c->setOrientation(Qt::Horizontal);
        _floor_label_d = new QLabel(centralwidget);
        _floor_label_d->setObjectName(QString::fromUtf8("_floor_label_d"));
        _floor_label_d->setGeometry(QRect(10, 450, 61, 16));
        _floor_horizontal_slider_d = new QSlider(centralwidget);
        _floor_horizontal_slider_d->setObjectName(QString::fromUtf8("_floor_horizontal_slider_d"));
        _floor_horizontal_slider_d->setGeometry(QRect(100, 450, 431, 20));
        _floor_horizontal_slider_d->setMinimum(0);
        _floor_horizontal_slider_d->setMaximum(1000);
        _floor_horizontal_slider_d->setValue(500);
        _floor_horizontal_slider_d->setOrientation(Qt::Horizontal);
        _floor_widget = new QWidget(centralwidget);
        _floor_widget->setObjectName(QString::fromUtf8("_floor_widget"));
        _floor_widget->setGeometry(QRect(290, 50, 240, 180));
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(80, 360, 20, 111));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 330, 521, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralwidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(260, 50, 20, 181));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        _floor_horizontal_slider_max = new QSlider(centralwidget);
        _floor_horizontal_slider_max->setObjectName(QString::fromUtf8("_floor_horizontal_slider_max"));
        _floor_horizontal_slider_max->setGeometry(QRect(100, 310, 431, 20));
        _floor_horizontal_slider_max->setMinimum(400);
        _floor_horizontal_slider_max->setMaximum(2000);
        _floor_horizontal_slider_max->setValue(1000);
        _floor_horizontal_slider_max->setOrientation(Qt::Horizontal);
        _floor_horizontal_slider_min = new QSlider(centralwidget);
        _floor_horizontal_slider_min->setObjectName(QString::fromUtf8("_floor_horizontal_slider_min"));
        _floor_horizontal_slider_min->setGeometry(QRect(100, 280, 431, 20));
        _floor_horizontal_slider_min->setMinimum(400);
        _floor_horizontal_slider_min->setMaximum(2000);
        _floor_horizontal_slider_min->setValue(500);
        _floor_horizontal_slider_min->setOrientation(Qt::Horizontal);
        line_4 = new QFrame(centralwidget);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setGeometry(QRect(10, 230, 521, 20));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        _floor_label_min = new QLabel(centralwidget);
        _floor_label_min->setObjectName(QString::fromUtf8("_floor_label_min"));
        _floor_label_min->setGeometry(QRect(10, 280, 81, 16));
        _floor_label_max = new QLabel(centralwidget);
        _floor_label_max->setObjectName(QString::fromUtf8("_floor_label_max"));
        _floor_label_max->setGeometry(QRect(10, 310, 81, 16));
        line_5 = new QFrame(centralwidget);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setGeometry(QRect(10, 30, 521, 20));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        _floor_push_button_add = new QPushButton(centralwidget);
        _floor_push_button_add->setObjectName(QString::fromUtf8("_floor_push_button_add"));
        _floor_push_button_add->setGeometry(QRect(10, 10, 75, 23));
        _floor_label_thresh = new QLabel(centralwidget);
        _floor_label_thresh->setObjectName(QString::fromUtf8("_floor_label_thresh"));
        _floor_label_thresh->setGeometry(QRect(10, 250, 81, 16));
        _floor_horizontal_slider_thresh = new QSlider(centralwidget);
        _floor_horizontal_slider_thresh->setObjectName(QString::fromUtf8("_floor_horizontal_slider_thresh"));
        _floor_horizontal_slider_thresh->setGeometry(QRect(100, 250, 431, 20));
        _floor_horizontal_slider_thresh->setMinimum(0);
        _floor_horizontal_slider_thresh->setMaximum(50);
        _floor_horizontal_slider_thresh->setValue(10);
        _floor_horizontal_slider_thresh->setOrientation(Qt::Horizontal);
        FloorManager->setCentralWidget(centralwidget);
        menubar = new QMenuBar(FloorManager);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 540, 18));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        FloorManager->setMenuBar(menubar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(_floor_action_close);

        retranslateUi(FloorManager);

        QMetaObject::connectSlotsByName(FloorManager);
    } // setupUi

    void retranslateUi(QMainWindow *FloorManager)
    {
        FloorManager->setWindowTitle(QApplication::translate("FloorManager", "MainWindow", 0, QApplication::UnicodeUTF8));
        _floor_action_close->setText(QApplication::translate("FloorManager", "Close", 0, QApplication::UnicodeUTF8));
        _floor_label_a->setText(QApplication::translate("FloorManager", "A:", 0, QApplication::UnicodeUTF8));
        _floor_label_b->setText(QApplication::translate("FloorManager", "B:", 0, QApplication::UnicodeUTF8));
        _floor_label_c->setText(QApplication::translate("FloorManager", "C:", 0, QApplication::UnicodeUTF8));
        _floor_label_d->setText(QApplication::translate("FloorManager", "D:", 0, QApplication::UnicodeUTF8));
        _floor_label_min->setText(QApplication::translate("FloorManager", "Min: 500mm", 0, QApplication::UnicodeUTF8));
        _floor_label_max->setText(QApplication::translate("FloorManager", "Max: 1000mm", 0, QApplication::UnicodeUTF8));
        _floor_push_button_add->setText(QApplication::translate("FloorManager", "Add Floor", 0, QApplication::UnicodeUTF8));
        _floor_label_thresh->setText(QApplication::translate("FloorManager", "Thresh: 10mm", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("FloorManager", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class FloorManager: public Ui_FloorManager {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FLOORMANAGERGUI_H
