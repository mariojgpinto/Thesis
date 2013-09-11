/********************************************************************************
** Form generated from reading UI file 'SpeculumGUI.ui'
**
** Created: Wed 11. Sep 20:16:31 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SPECULUMGUI_H
#define UI_SPECULUMGUI_H

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

class Ui_SpeculumGUI
{
public:
    QAction *_action_close;
    QAction *_action_save;
    QAction *_action_load;
    QWidget *centralWidget;
    QWidget *_main_widget;
    QPushButton *_main_push_button_pause;
    QFrame *line;
    QPushButton *_main_push_button_mirror_manager;
    QLabel *_main_label_min;
    QLabel *_main_label_max;
    QFrame *line_2;
    QSlider *_main_horizontalSlider_min;
    QSlider *_main_horizontalSlider_max;
    QPushButton *_main_push_button_floor_manager;
    QMenuBar *menuBar;
    QMenu *menuFile;

    void setupUi(QMainWindow *SpeculumGUI)
    {
        if (SpeculumGUI->objectName().isEmpty())
            SpeculumGUI->setObjectName(QString::fromUtf8("SpeculumGUI"));
        SpeculumGUI->resize(660, 625);
        _action_close = new QAction(SpeculumGUI);
        _action_close->setObjectName(QString::fromUtf8("_action_close"));
        _action_save = new QAction(SpeculumGUI);
        _action_save->setObjectName(QString::fromUtf8("_action_save"));
        _action_load = new QAction(SpeculumGUI);
        _action_load->setObjectName(QString::fromUtf8("_action_load"));
        centralWidget = new QWidget(SpeculumGUI);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        _main_widget = new QWidget(centralWidget);
        _main_widget->setObjectName(QString::fromUtf8("_main_widget"));
        _main_widget->setGeometry(QRect(10, 40, 640, 480));
        _main_push_button_pause = new QPushButton(centralWidget);
        _main_push_button_pause->setObjectName(QString::fromUtf8("_main_push_button_pause"));
        _main_push_button_pause->setGeometry(QRect(570, 10, 75, 23));
        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(200, 10, 20, 21));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        _main_push_button_mirror_manager = new QPushButton(centralWidget);
        _main_push_button_mirror_manager->setObjectName(QString::fromUtf8("_main_push_button_mirror_manager"));
        _main_push_button_mirror_manager->setGeometry(QRect(110, 10, 81, 23));
        _main_label_min = new QLabel(centralWidget);
        _main_label_min->setObjectName(QString::fromUtf8("_main_label_min"));
        _main_label_min->setGeometry(QRect(10, 540, 81, 16));
        _main_label_max = new QLabel(centralWidget);
        _main_label_max->setObjectName(QString::fromUtf8("_main_label_max"));
        _main_label_max->setGeometry(QRect(10, 570, 81, 16));
        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 520, 641, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        _main_horizontalSlider_min = new QSlider(centralWidget);
        _main_horizontalSlider_min->setObjectName(QString::fromUtf8("_main_horizontalSlider_min"));
        _main_horizontalSlider_min->setGeometry(QRect(100, 540, 551, 20));
        _main_horizontalSlider_min->setMinimum(400);
        _main_horizontalSlider_min->setMaximum(2000);
        _main_horizontalSlider_min->setValue(500);
        _main_horizontalSlider_min->setOrientation(Qt::Horizontal);
        _main_horizontalSlider_max = new QSlider(centralWidget);
        _main_horizontalSlider_max->setObjectName(QString::fromUtf8("_main_horizontalSlider_max"));
        _main_horizontalSlider_max->setGeometry(QRect(100, 570, 551, 20));
        _main_horizontalSlider_max->setMinimum(400);
        _main_horizontalSlider_max->setMaximum(2000);
        _main_horizontalSlider_max->setValue(1000);
        _main_horizontalSlider_max->setOrientation(Qt::Horizontal);
        _main_push_button_floor_manager = new QPushButton(centralWidget);
        _main_push_button_floor_manager->setObjectName(QString::fromUtf8("_main_push_button_floor_manager"));
        _main_push_button_floor_manager->setGeometry(QRect(10, 10, 81, 23));
        SpeculumGUI->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(SpeculumGUI);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 660, 18));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        SpeculumGUI->setMenuBar(menuBar);

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
        _main_push_button_pause->setText(QApplication::translate("SpeculumGUI", "Pause", 0, QApplication::UnicodeUTF8));
        _main_push_button_mirror_manager->setText(QApplication::translate("SpeculumGUI", "MirrorManager", 0, QApplication::UnicodeUTF8));
        _main_label_min->setText(QApplication::translate("SpeculumGUI", "Min: 500mm", 0, QApplication::UnicodeUTF8));
        _main_label_max->setText(QApplication::translate("SpeculumGUI", "Max: 1000mm", 0, QApplication::UnicodeUTF8));
        _main_push_button_floor_manager->setText(QApplication::translate("SpeculumGUI", "FloorManager", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("SpeculumGUI", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SpeculumGUI: public Ui_SpeculumGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SPECULUMGUI_H
