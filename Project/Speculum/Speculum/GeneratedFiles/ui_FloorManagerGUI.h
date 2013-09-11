/********************************************************************************
** Form generated from reading UI file 'FloorManagerGUI.ui'
**
** Created: Wed 11. Sep 20:31:18 2013
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
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FloorManagerGUI
{
public:
    QAction *_floor_action_close;
    QAction *_floor_action_save;
    QWidget *centralwidget;
    QWidget *_floor_widget;
    QFrame *line;
    QGroupBox *groupBox;
    QLabel *label;
    QCheckBox *_floor_check_box_input_manual;
    QLabel *label_2;
    QCheckBox *_floor_check_box_input_automatic;
    QPushButton *_floor_push_button_add_floor_normal;
    QFrame *line_2;
    QDoubleSpinBox *_floor_spin_box_double_thresh;
    QPushButton *_floor_push_button_save;
    QLabel *label_3;
    QPushButton *_floor_push_button_set_area;
    QFrame *line_3;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *FloorManagerGUI)
    {
        if (FloorManagerGUI->objectName().isEmpty())
            FloorManagerGUI->setObjectName(QString::fromUtf8("FloorManagerGUI"));
        FloorManagerGUI->resize(829, 535);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(FloorManagerGUI->sizePolicy().hasHeightForWidth());
        FloorManagerGUI->setSizePolicy(sizePolicy);
        FloorManagerGUI->setMinimumSize(QSize(829, 535));
        FloorManagerGUI->setMaximumSize(QSize(829, 535));
        _floor_action_close = new QAction(FloorManagerGUI);
        _floor_action_close->setObjectName(QString::fromUtf8("_floor_action_close"));
        _floor_action_save = new QAction(FloorManagerGUI);
        _floor_action_save->setObjectName(QString::fromUtf8("_floor_action_save"));
        centralwidget = new QWidget(FloorManagerGUI);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        _floor_widget = new QWidget(centralwidget);
        _floor_widget->setObjectName(QString::fromUtf8("_floor_widget"));
        _floor_widget->setGeometry(QRect(10, 10, 640, 480));
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(650, 10, 20, 481));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(670, 10, 151, 481));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 200, 111, 16));
        _floor_check_box_input_manual = new QCheckBox(groupBox);
        _floor_check_box_input_manual->setObjectName(QString::fromUtf8("_floor_check_box_input_manual"));
        _floor_check_box_input_manual->setGeometry(QRect(10, 110, 70, 17));
        _floor_check_box_input_manual->setChecked(true);
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 90, 101, 16));
        _floor_check_box_input_automatic = new QCheckBox(groupBox);
        _floor_check_box_input_automatic->setObjectName(QString::fromUtf8("_floor_check_box_input_automatic"));
        _floor_check_box_input_automatic->setEnabled(false);
        _floor_check_box_input_automatic->setGeometry(QRect(10, 130, 70, 17));
        _floor_push_button_add_floor_normal = new QPushButton(groupBox);
        _floor_push_button_add_floor_normal->setObjectName(QString::fromUtf8("_floor_push_button_add_floor_normal"));
        _floor_push_button_add_floor_normal->setGeometry(QRect(10, 150, 121, 23));
        line_2 = new QFrame(groupBox);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 180, 131, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        _floor_spin_box_double_thresh = new QDoubleSpinBox(groupBox);
        _floor_spin_box_double_thresh->setObjectName(QString::fromUtf8("_floor_spin_box_double_thresh"));
        _floor_spin_box_double_thresh->setGeometry(QRect(10, 220, 62, 22));
        _floor_spin_box_double_thresh->setSingleStep(0.005);
        _floor_spin_box_double_thresh->setValue(0.03);
        _floor_push_button_save = new QPushButton(groupBox);
        _floor_push_button_save->setObjectName(QString::fromUtf8("_floor_push_button_save"));
        _floor_push_button_save->setGeometry(QRect(14, 450, 121, 23));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 20, 101, 16));
        _floor_push_button_set_area = new QPushButton(groupBox);
        _floor_push_button_set_area->setObjectName(QString::fromUtf8("_floor_push_button_set_area"));
        _floor_push_button_set_area->setGeometry(QRect(10, 40, 75, 23));
        line_3 = new QFrame(groupBox);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(10, 70, 131, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        FloorManagerGUI->setCentralWidget(centralwidget);
        menubar = new QMenuBar(FloorManagerGUI);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 829, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        FloorManagerGUI->setMenuBar(menubar);
        statusbar = new QStatusBar(FloorManagerGUI);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        FloorManagerGUI->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(_floor_action_save);
        menuFile->addSeparator();
        menuFile->addAction(_floor_action_close);

        retranslateUi(FloorManagerGUI);

        QMetaObject::connectSlotsByName(FloorManagerGUI);
    } // setupUi

    void retranslateUi(QMainWindow *FloorManagerGUI)
    {
        FloorManagerGUI->setWindowTitle(QApplication::translate("FloorManagerGUI", "MainWindow", 0, QApplication::UnicodeUTF8));
        _floor_action_close->setText(QApplication::translate("FloorManagerGUI", "Close", 0, QApplication::UnicodeUTF8));
        _floor_action_save->setText(QApplication::translate("FloorManagerGUI", "Save", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("FloorManagerGUI", "Setup", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("FloorManagerGUI", "Floor Threshold (mm):", 0, QApplication::UnicodeUTF8));
        _floor_check_box_input_manual->setText(QApplication::translate("FloorManagerGUI", "Manual", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("FloorManagerGUI", "Set Floor Normal:", 0, QApplication::UnicodeUTF8));
        _floor_check_box_input_automatic->setText(QApplication::translate("FloorManagerGUI", "Automatic", 0, QApplication::UnicodeUTF8));
        _floor_push_button_add_floor_normal->setText(QApplication::translate("FloorManagerGUI", "Add Floor Normal", 0, QApplication::UnicodeUTF8));
        _floor_push_button_save->setText(QApplication::translate("FloorManagerGUI", "Save", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("FloorManagerGUI", "Set Floor Area:", 0, QApplication::UnicodeUTF8));
        _floor_push_button_set_area->setText(QApplication::translate("FloorManagerGUI", "Set Area", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("FloorManagerGUI", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class FloorManagerGUI: public Ui_FloorManagerGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FLOORMANAGERGUI_H
