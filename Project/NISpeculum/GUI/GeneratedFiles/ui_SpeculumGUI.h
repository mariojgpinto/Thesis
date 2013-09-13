/********************************************************************************
** Form generated from reading UI file 'SpeculumGUI.ui'
**
** Created: Fri 13. Sep 16:56:41 2013
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
#include <QtGui/QSpinBox>
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
    QFrame *line_2;
    QPushButton *_main_push_button_floor_manager;
    QSpinBox *_main_spin_box_pcl;
    QLabel *label;
    QFrame *line_3;
    QPushButton *_main_push_button_3d_manager;
    QMenuBar *menuBar;
    QMenu *menuFile;

    void setupUi(QMainWindow *SpeculumGUI)
    {
        if (SpeculumGUI->objectName().isEmpty())
            SpeculumGUI->setObjectName(QString::fromUtf8("SpeculumGUI"));
        SpeculumGUI->resize(660, 580);
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
        _main_widget->setGeometry(QRect(10, 60, 640, 480));
        _main_push_button_pause = new QPushButton(centralWidget);
        _main_push_button_pause->setObjectName(QString::fromUtf8("_main_push_button_pause"));
        _main_push_button_pause->setGeometry(QRect(480, 10, 75, 23));
        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(200, 10, 20, 21));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        _main_push_button_mirror_manager = new QPushButton(centralWidget);
        _main_push_button_mirror_manager->setObjectName(QString::fromUtf8("_main_push_button_mirror_manager"));
        _main_push_button_mirror_manager->setGeometry(QRect(110, 10, 81, 23));
        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 40, 641, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        _main_push_button_floor_manager = new QPushButton(centralWidget);
        _main_push_button_floor_manager->setObjectName(QString::fromUtf8("_main_push_button_floor_manager"));
        _main_push_button_floor_manager->setGeometry(QRect(10, 10, 81, 23));
        _main_spin_box_pcl = new QSpinBox(centralWidget);
        _main_spin_box_pcl->setObjectName(QString::fromUtf8("_main_spin_box_pcl"));
        _main_spin_box_pcl->setGeometry(QRect(611, 10, 41, 22));
        _main_spin_box_pcl->setMinimum(1);
        _main_spin_box_pcl->setMaximum(16);
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(580, 10, 31, 20));
        line_3 = new QFrame(centralWidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(560, 10, 20, 21));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        _main_push_button_3d_manager = new QPushButton(centralWidget);
        _main_push_button_3d_manager->setObjectName(QString::fromUtf8("_main_push_button_3d_manager"));
        _main_push_button_3d_manager->setGeometry(QRect(230, 10, 81, 23));
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
        _main_push_button_mirror_manager->setText(QApplication::translate("SpeculumGUI", "Mirror Manager", 0, QApplication::UnicodeUTF8));
        _main_push_button_floor_manager->setText(QApplication::translate("SpeculumGUI", "Floor Manager", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SpeculumGUI", "Step:", 0, QApplication::UnicodeUTF8));
        _main_push_button_3d_manager->setText(QApplication::translate("SpeculumGUI", "3D Manager", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("SpeculumGUI", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SpeculumGUI: public Ui_SpeculumGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SPECULUMGUI_H
