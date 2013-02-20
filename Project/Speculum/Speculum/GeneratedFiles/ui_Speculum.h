/********************************************************************************
** Form generated from reading UI file 'Speculum.ui'
**
** Created: Tue 19. Feb 22:53:32 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SPECULUM_H
#define UI_SPECULUM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SpeculumClass
{
public:
    QAction *_main_action_save_model;
    QAction *_main_action_exit;
    QAction *_main_action_save_configuration;
    QAction *_main_action_load_configuration;
    QAction *_main_action_preferences;
    QAction *_main_action_source_kinect;
    QAction *_main_action_source_file;
    QAction *_main_action_add_mirror;
    QAction *_main_action_manage_mirrors;
    QAction *_main_action_manager_floor;
    QWidget *centralWidget;
    QWidget *_main_widget_left;
    QWidget *_main_widget_right1;
    QPushButton *_main_push_button_floor;
    QFrame *line;
    QWidget *_main_widget_right2;
    QPushButton *_main_push_button_mirrors;
    QPushButton *_main_push_button_object;
    QFrame *line_2;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuSource;
    QMenu *menuConfiguration;
    QMenu *menuPreferences;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *SpeculumClass)
    {
        if (SpeculumClass->objectName().isEmpty())
            SpeculumClass->setObjectName(QString::fromUtf8("SpeculumClass"));
        SpeculumClass->resize(920, 583);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SpeculumClass->sizePolicy().hasHeightForWidth());
        SpeculumClass->setSizePolicy(sizePolicy);
        SpeculumClass->setMinimumSize(QSize(920, 583));
        SpeculumClass->setMaximumSize(QSize(920, 583));
        _main_action_save_model = new QAction(SpeculumClass);
        _main_action_save_model->setObjectName(QString::fromUtf8("_main_action_save_model"));
        _main_action_exit = new QAction(SpeculumClass);
        _main_action_exit->setObjectName(QString::fromUtf8("_main_action_exit"));
        _main_action_save_configuration = new QAction(SpeculumClass);
        _main_action_save_configuration->setObjectName(QString::fromUtf8("_main_action_save_configuration"));
        _main_action_load_configuration = new QAction(SpeculumClass);
        _main_action_load_configuration->setObjectName(QString::fromUtf8("_main_action_load_configuration"));
        _main_action_preferences = new QAction(SpeculumClass);
        _main_action_preferences->setObjectName(QString::fromUtf8("_main_action_preferences"));
        _main_action_source_kinect = new QAction(SpeculumClass);
        _main_action_source_kinect->setObjectName(QString::fromUtf8("_main_action_source_kinect"));
        _main_action_source_file = new QAction(SpeculumClass);
        _main_action_source_file->setObjectName(QString::fromUtf8("_main_action_source_file"));
        _main_action_add_mirror = new QAction(SpeculumClass);
        _main_action_add_mirror->setObjectName(QString::fromUtf8("_main_action_add_mirror"));
        _main_action_manage_mirrors = new QAction(SpeculumClass);
        _main_action_manage_mirrors->setObjectName(QString::fromUtf8("_main_action_manage_mirrors"));
        _main_action_manager_floor = new QAction(SpeculumClass);
        _main_action_manager_floor->setObjectName(QString::fromUtf8("_main_action_manager_floor"));
        centralWidget = new QWidget(SpeculumClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        _main_widget_left = new QWidget(centralWidget);
        _main_widget_left->setObjectName(QString::fromUtf8("_main_widget_left"));
        _main_widget_left->setGeometry(QRect(10, 10, 640, 480));
        _main_widget_right1 = new QWidget(centralWidget);
        _main_widget_right1->setObjectName(QString::fromUtf8("_main_widget_right1"));
        _main_widget_right1->setGeometry(QRect(670, 10, 240, 160));
        _main_push_button_floor = new QPushButton(centralWidget);
        _main_push_button_floor->setObjectName(QString::fromUtf8("_main_push_button_floor"));
        _main_push_button_floor->setGeometry(QRect(10, 500, 75, 23));
        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(650, 10, 20, 481));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        _main_widget_right2 = new QWidget(centralWidget);
        _main_widget_right2->setObjectName(QString::fromUtf8("_main_widget_right2"));
        _main_widget_right2->setGeometry(QRect(670, 190, 240, 160));
        _main_push_button_mirrors = new QPushButton(centralWidget);
        _main_push_button_mirrors->setObjectName(QString::fromUtf8("_main_push_button_mirrors"));
        _main_push_button_mirrors->setGeometry(QRect(100, 500, 75, 23));
        _main_push_button_object = new QPushButton(centralWidget);
        _main_push_button_object->setObjectName(QString::fromUtf8("_main_push_button_object"));
        _main_push_button_object->setGeometry(QRect(210, 500, 75, 23));
        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(180, 500, 20, 21));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);
        SpeculumClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(SpeculumClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 920, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuSource = new QMenu(menuFile);
        menuSource->setObjectName(QString::fromUtf8("menuSource"));
        menuConfiguration = new QMenu(menuBar);
        menuConfiguration->setObjectName(QString::fromUtf8("menuConfiguration"));
        menuPreferences = new QMenu(menuBar);
        menuPreferences->setObjectName(QString::fromUtf8("menuPreferences"));
        SpeculumClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(SpeculumClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        SpeculumClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(SpeculumClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        SpeculumClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuConfiguration->menuAction());
        menuBar->addAction(menuPreferences->menuAction());
        menuFile->addAction(menuSource->menuAction());
        menuFile->addAction(_main_action_save_model);
        menuFile->addSeparator();
        menuFile->addAction(_main_action_exit);
        menuSource->addAction(_main_action_source_kinect);
        menuSource->addAction(_main_action_source_file);
        menuConfiguration->addAction(_main_action_manage_mirrors);
        menuConfiguration->addSeparator();
        menuConfiguration->addAction(_main_action_manager_floor);
        menuConfiguration->addSeparator();
        menuConfiguration->addAction(_main_action_save_configuration);
        menuConfiguration->addAction(_main_action_load_configuration);
        menuPreferences->addAction(_main_action_preferences);

        retranslateUi(SpeculumClass);

        QMetaObject::connectSlotsByName(SpeculumClass);
    } // setupUi

    void retranslateUi(QMainWindow *SpeculumClass)
    {
        SpeculumClass->setWindowTitle(QApplication::translate("SpeculumClass", "Speculum", 0, QApplication::UnicodeUTF8));
        _main_action_save_model->setText(QApplication::translate("SpeculumClass", "Save Model", 0, QApplication::UnicodeUTF8));
        _main_action_exit->setText(QApplication::translate("SpeculumClass", "Exit", 0, QApplication::UnicodeUTF8));
        _main_action_save_configuration->setText(QApplication::translate("SpeculumClass", "Save Configuration", 0, QApplication::UnicodeUTF8));
        _main_action_load_configuration->setText(QApplication::translate("SpeculumClass", "Load Configuration", 0, QApplication::UnicodeUTF8));
        _main_action_preferences->setText(QApplication::translate("SpeculumClass", "Preferences", 0, QApplication::UnicodeUTF8));
        _main_action_source_kinect->setText(QApplication::translate("SpeculumClass", "Kinect", 0, QApplication::UnicodeUTF8));
        _main_action_source_file->setText(QApplication::translate("SpeculumClass", "File", 0, QApplication::UnicodeUTF8));
        _main_action_add_mirror->setText(QApplication::translate("SpeculumClass", "Add Mirror", 0, QApplication::UnicodeUTF8));
        _main_action_manage_mirrors->setText(QApplication::translate("SpeculumClass", "Manage Mirrors", 0, QApplication::UnicodeUTF8));
        _main_action_manager_floor->setText(QApplication::translate("SpeculumClass", "Manage Floor", 0, QApplication::UnicodeUTF8));
        _main_push_button_floor->setText(QApplication::translate("SpeculumClass", "Calc Floor", 0, QApplication::UnicodeUTF8));
        _main_push_button_mirrors->setText(QApplication::translate("SpeculumClass", "Floor", 0, QApplication::UnicodeUTF8));
        _main_push_button_object->setText(QApplication::translate("SpeculumClass", "Object", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("SpeculumClass", "File", 0, QApplication::UnicodeUTF8));
        menuSource->setTitle(QApplication::translate("SpeculumClass", "Source", 0, QApplication::UnicodeUTF8));
        menuConfiguration->setTitle(QApplication::translate("SpeculumClass", "Configuration", 0, QApplication::UnicodeUTF8));
        menuPreferences->setTitle(QApplication::translate("SpeculumClass", "Preferences", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SpeculumClass: public Ui_SpeculumClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SPECULUM_H
