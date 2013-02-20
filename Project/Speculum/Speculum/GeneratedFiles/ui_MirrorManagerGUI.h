/********************************************************************************
** Form generated from reading UI file 'MirrorManagerGUI.ui'
**
** Created: Tue 19. Feb 22:53:33 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MIRRORMANAGERGUI_H
#define UI_MIRRORMANAGERGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MirrorManagerGUI
{
public:
    QAction *_mirror_action_close;
    QAction *_mirror_action_add_mirror;
    QWidget *centralwidget;
    QTabWidget *mirror_tab_widget;
    QWidget *_mirror_tab_view;
    QWidget *_mirror_view_widget_left;
    QWidget *_mirror_view_widget3;
    QWidget *_mirror_view_widget4;
    QWidget *_mirror_view_widget2;
    QWidget *_mirror_view_widget1;
    QWidget *_mirror_tab_edit;
    QWidget *_mirror_widget_bottom_right;
    QSpinBox *_mirror_spinbox_mirror;
    QGroupBox *mirror_groupbox_data;
    QLabel *_mirror_label_vertix;
    QSpinBox *_mirror_spinbox_vertix;
    QSpinBox *_mirror_spinbox_vertix_x;
    QFrame *line_2;
    QLabel *_mirror_label_posx;
    QFrame *line_3;
    QSpinBox *_mirror_spinbox_vertix_y;
    QLabel *_mirror_label_posy;
    QFrame *line_5;
    QSpinBox *_mirror_spinbox_vertix_x_axis;
    QLabel *_mirror_label_x_axis;
    QSpinBox *_mirror_spinbox_vertix_y_axis;
    QLabel *_mirror_label_y_axis;
    QSpinBox *_mirror_spinbox_vertix_z_axis;
    QLabel *_mirror_label_z_axis;
    QFrame *line_6;
    QFrame *line_7;
    QFrame *line_8;
    QLabel *_mirror_label_distance;
    QSlider *_mirror_slider_distance;
    QPushButton *_mirror_push_button_calc_points;
    QPushButton *pushButton_2;
    QFrame *line_4;
    QWidget *_mirror_widget_top;
    QWidget *_mirror_widget_bottom_left;
    QLabel *_mirror_label_mirror;
    QPushButton *_mirror_push_button_add_mirror;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuConfiguration;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MirrorManagerGUI)
    {
        if (MirrorManagerGUI->objectName().isEmpty())
            MirrorManagerGUI->setObjectName(QString::fromUtf8("MirrorManagerGUI"));
        MirrorManagerGUI->resize(790, 620);
        _mirror_action_close = new QAction(MirrorManagerGUI);
        _mirror_action_close->setObjectName(QString::fromUtf8("_mirror_action_close"));
        _mirror_action_add_mirror = new QAction(MirrorManagerGUI);
        _mirror_action_add_mirror->setObjectName(QString::fromUtf8("_mirror_action_add_mirror"));
        centralwidget = new QWidget(MirrorManagerGUI);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        mirror_tab_widget = new QTabWidget(centralwidget);
        mirror_tab_widget->setObjectName(QString::fromUtf8("mirror_tab_widget"));
        mirror_tab_widget->setGeometry(QRect(10, 10, 771, 561));
        _mirror_tab_view = new QWidget();
        _mirror_tab_view->setObjectName(QString::fromUtf8("_mirror_tab_view"));
        _mirror_view_widget_left = new QWidget(_mirror_tab_view);
        _mirror_view_widget_left->setObjectName(QString::fromUtf8("_mirror_view_widget_left"));
        _mirror_view_widget_left->setGeometry(QRect(10, 10, 480, 369));
        _mirror_view_widget3 = new QWidget(_mirror_tab_view);
        _mirror_view_widget3->setObjectName(QString::fromUtf8("_mirror_view_widget3"));
        _mirror_view_widget3->setGeometry(QRect(510, 10, 240, 160));
        _mirror_view_widget4 = new QWidget(_mirror_tab_view);
        _mirror_view_widget4->setObjectName(QString::fromUtf8("_mirror_view_widget4"));
        _mirror_view_widget4->setGeometry(QRect(510, 180, 240, 160));
        _mirror_view_widget2 = new QWidget(_mirror_tab_view);
        _mirror_view_widget2->setObjectName(QString::fromUtf8("_mirror_view_widget2"));
        _mirror_view_widget2->setGeometry(QRect(570, 350, 120, 80));
        _mirror_view_widget1 = new QWidget(_mirror_tab_view);
        _mirror_view_widget1->setObjectName(QString::fromUtf8("_mirror_view_widget1"));
        _mirror_view_widget1->setGeometry(QRect(570, 440, 120, 80));
        mirror_tab_widget->addTab(_mirror_tab_view, QString());
        _mirror_tab_edit = new QWidget();
        _mirror_tab_edit->setObjectName(QString::fromUtf8("_mirror_tab_edit"));
        _mirror_widget_bottom_right = new QWidget(_mirror_tab_edit);
        _mirror_widget_bottom_right->setObjectName(QString::fromUtf8("_mirror_widget_bottom_right"));
        _mirror_widget_bottom_right->setGeometry(QRect(260, 370, 240, 160));
        _mirror_spinbox_mirror = new QSpinBox(_mirror_tab_edit);
        _mirror_spinbox_mirror->setObjectName(QString::fromUtf8("_mirror_spinbox_mirror"));
        _mirror_spinbox_mirror->setGeometry(QRect(50, 10, 51, 22));
        mirror_groupbox_data = new QGroupBox(_mirror_tab_edit);
        mirror_groupbox_data->setObjectName(QString::fromUtf8("mirror_groupbox_data"));
        mirror_groupbox_data->setGeometry(QRect(520, 40, 241, 491));
        _mirror_label_vertix = new QLabel(mirror_groupbox_data);
        _mirror_label_vertix->setObjectName(QString::fromUtf8("_mirror_label_vertix"));
        _mirror_label_vertix->setGeometry(QRect(100, 20, 51, 21));
        _mirror_label_vertix->setAlignment(Qt::AlignCenter);
        _mirror_spinbox_vertix = new QSpinBox(mirror_groupbox_data);
        _mirror_spinbox_vertix->setObjectName(QString::fromUtf8("_mirror_spinbox_vertix"));
        _mirror_spinbox_vertix->setGeometry(QRect(100, 40, 51, 22));
        _mirror_spinbox_vertix->setMaximum(0);
        _mirror_spinbox_vertix_x = new QSpinBox(mirror_groupbox_data);
        _mirror_spinbox_vertix_x->setObjectName(QString::fromUtf8("_mirror_spinbox_vertix_x"));
        _mirror_spinbox_vertix_x->setGeometry(QRect(50, 100, 51, 22));
        _mirror_spinbox_vertix_x->setMaximum(640);
        line_2 = new QFrame(mirror_groupbox_data);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 60, 221, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        _mirror_label_posx = new QLabel(mirror_groupbox_data);
        _mirror_label_posx->setObjectName(QString::fromUtf8("_mirror_label_posx"));
        _mirror_label_posx->setGeometry(QRect(50, 80, 46, 16));
        line_3 = new QFrame(mirror_groupbox_data);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(10, 130, 221, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        _mirror_spinbox_vertix_y = new QSpinBox(mirror_groupbox_data);
        _mirror_spinbox_vertix_y->setObjectName(QString::fromUtf8("_mirror_spinbox_vertix_y"));
        _mirror_spinbox_vertix_y->setGeometry(QRect(140, 100, 51, 22));
        _mirror_spinbox_vertix_y->setMaximum(480);
        _mirror_label_posy = new QLabel(mirror_groupbox_data);
        _mirror_label_posy->setObjectName(QString::fromUtf8("_mirror_label_posy"));
        _mirror_label_posy->setGeometry(QRect(140, 80, 46, 16));
        line_5 = new QFrame(mirror_groupbox_data);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setGeometry(QRect(110, 80, 20, 51));
        line_5->setFrameShape(QFrame::VLine);
        line_5->setFrameShadow(QFrame::Sunken);
        _mirror_spinbox_vertix_x_axis = new QSpinBox(mirror_groupbox_data);
        _mirror_spinbox_vertix_x_axis->setObjectName(QString::fromUtf8("_mirror_spinbox_vertix_x_axis"));
        _mirror_spinbox_vertix_x_axis->setGeometry(QRect(30, 170, 51, 22));
        _mirror_label_x_axis = new QLabel(mirror_groupbox_data);
        _mirror_label_x_axis->setObjectName(QString::fromUtf8("_mirror_label_x_axis"));
        _mirror_label_x_axis->setGeometry(QRect(30, 150, 46, 16));
        _mirror_label_x_axis->setAlignment(Qt::AlignCenter);
        _mirror_spinbox_vertix_y_axis = new QSpinBox(mirror_groupbox_data);
        _mirror_spinbox_vertix_y_axis->setObjectName(QString::fromUtf8("_mirror_spinbox_vertix_y_axis"));
        _mirror_spinbox_vertix_y_axis->setGeometry(QRect(100, 170, 51, 22));
        _mirror_label_y_axis = new QLabel(mirror_groupbox_data);
        _mirror_label_y_axis->setObjectName(QString::fromUtf8("_mirror_label_y_axis"));
        _mirror_label_y_axis->setGeometry(QRect(100, 150, 46, 16));
        _mirror_label_y_axis->setAlignment(Qt::AlignCenter);
        _mirror_spinbox_vertix_z_axis = new QSpinBox(mirror_groupbox_data);
        _mirror_spinbox_vertix_z_axis->setObjectName(QString::fromUtf8("_mirror_spinbox_vertix_z_axis"));
        _mirror_spinbox_vertix_z_axis->setGeometry(QRect(170, 170, 51, 22));
        _mirror_label_z_axis = new QLabel(mirror_groupbox_data);
        _mirror_label_z_axis->setObjectName(QString::fromUtf8("_mirror_label_z_axis"));
        _mirror_label_z_axis->setGeometry(QRect(170, 150, 46, 16));
        _mirror_label_z_axis->setAlignment(Qt::AlignCenter);
        line_6 = new QFrame(mirror_groupbox_data);
        line_6->setObjectName(QString::fromUtf8("line_6"));
        line_6->setGeometry(QRect(80, 150, 20, 51));
        line_6->setFrameShape(QFrame::VLine);
        line_6->setFrameShadow(QFrame::Sunken);
        line_7 = new QFrame(mirror_groupbox_data);
        line_7->setObjectName(QString::fromUtf8("line_7"));
        line_7->setGeometry(QRect(150, 150, 20, 51));
        line_7->setFrameShape(QFrame::VLine);
        line_7->setFrameShadow(QFrame::Sunken);
        line_8 = new QFrame(mirror_groupbox_data);
        line_8->setObjectName(QString::fromUtf8("line_8"));
        line_8->setGeometry(QRect(10, 270, 221, 16));
        line_8->setFrameShape(QFrame::HLine);
        line_8->setFrameShadow(QFrame::Sunken);
        _mirror_label_distance = new QLabel(mirror_groupbox_data);
        _mirror_label_distance->setObjectName(QString::fromUtf8("_mirror_label_distance"));
        _mirror_label_distance->setGeometry(QRect(10, 210, 91, 16));
        _mirror_slider_distance = new QSlider(mirror_groupbox_data);
        _mirror_slider_distance->setObjectName(QString::fromUtf8("_mirror_slider_distance"));
        _mirror_slider_distance->setGeometry(QRect(10, 240, 221, 19));
        _mirror_slider_distance->setOrientation(Qt::Horizontal);
        _mirror_push_button_calc_points = new QPushButton(mirror_groupbox_data);
        _mirror_push_button_calc_points->setObjectName(QString::fromUtf8("_mirror_push_button_calc_points"));
        _mirror_push_button_calc_points->setGeometry(QRect(20, 290, 211, 23));
        pushButton_2 = new QPushButton(mirror_groupbox_data);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setEnabled(false);
        pushButton_2->setGeometry(QRect(20, 320, 211, 23));
        line_4 = new QFrame(_mirror_tab_edit);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setGeometry(QRect(500, 40, 16, 491));
        line_4->setFrameShape(QFrame::VLine);
        line_4->setFrameShadow(QFrame::Sunken);
        _mirror_widget_top = new QWidget(_mirror_tab_edit);
        _mirror_widget_top->setObjectName(QString::fromUtf8("_mirror_widget_top"));
        _mirror_widget_top->setGeometry(QRect(10, 40, 480, 320));
        _mirror_widget_bottom_left = new QWidget(_mirror_tab_edit);
        _mirror_widget_bottom_left->setObjectName(QString::fromUtf8("_mirror_widget_bottom_left"));
        _mirror_widget_bottom_left->setGeometry(QRect(10, 370, 240, 160));
        _mirror_label_mirror = new QLabel(_mirror_tab_edit);
        _mirror_label_mirror->setObjectName(QString::fromUtf8("_mirror_label_mirror"));
        _mirror_label_mirror->setGeometry(QRect(10, 10, 41, 21));
        _mirror_push_button_add_mirror = new QPushButton(_mirror_tab_edit);
        _mirror_push_button_add_mirror->setObjectName(QString::fromUtf8("_mirror_push_button_add_mirror"));
        _mirror_push_button_add_mirror->setGeometry(QRect(120, 10, 75, 23));
        mirror_tab_widget->addTab(_mirror_tab_edit, QString());
        MirrorManagerGUI->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MirrorManagerGUI);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 790, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuConfiguration = new QMenu(menubar);
        menuConfiguration->setObjectName(QString::fromUtf8("menuConfiguration"));
        MirrorManagerGUI->setMenuBar(menubar);
        statusbar = new QStatusBar(MirrorManagerGUI);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MirrorManagerGUI->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuConfiguration->menuAction());
        menuFile->addAction(_mirror_action_close);
        menuConfiguration->addAction(_mirror_action_add_mirror);

        retranslateUi(MirrorManagerGUI);

        mirror_tab_widget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MirrorManagerGUI);
    } // setupUi

    void retranslateUi(QMainWindow *MirrorManagerGUI)
    {
        MirrorManagerGUI->setWindowTitle(QApplication::translate("MirrorManagerGUI", "MainWindow", 0, QApplication::UnicodeUTF8));
        _mirror_action_close->setText(QApplication::translate("MirrorManagerGUI", "Close", 0, QApplication::UnicodeUTF8));
        _mirror_action_add_mirror->setText(QApplication::translate("MirrorManagerGUI", "Add Mirror", 0, QApplication::UnicodeUTF8));
        mirror_tab_widget->setTabText(mirror_tab_widget->indexOf(_mirror_tab_view), QApplication::translate("MirrorManagerGUI", "View", 0, QApplication::UnicodeUTF8));
        mirror_groupbox_data->setTitle(QApplication::translate("MirrorManagerGUI", "Mirror", 0, QApplication::UnicodeUTF8));
        _mirror_label_vertix->setText(QApplication::translate("MirrorManagerGUI", "Vertix:", 0, QApplication::UnicodeUTF8));
        _mirror_label_posx->setText(QApplication::translate("MirrorManagerGUI", "Position X:", 0, QApplication::UnicodeUTF8));
        _mirror_label_posy->setText(QApplication::translate("MirrorManagerGUI", "Position Y:", 0, QApplication::UnicodeUTF8));
        _mirror_label_x_axis->setText(QApplication::translate("MirrorManagerGUI", "X-Axis", 0, QApplication::UnicodeUTF8));
        _mirror_label_y_axis->setText(QApplication::translate("MirrorManagerGUI", "Y-Axis", 0, QApplication::UnicodeUTF8));
        _mirror_label_z_axis->setText(QApplication::translate("MirrorManagerGUI", "Z-Axis", 0, QApplication::UnicodeUTF8));
        _mirror_label_distance->setText(QApplication::translate("MirrorManagerGUI", "Distance:", 0, QApplication::UnicodeUTF8));
        _mirror_push_button_calc_points->setText(QApplication::translate("MirrorManagerGUI", "Calc Mirror Plane by Points", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("MirrorManagerGUI", "Calc Mirror Plane by Mark", 0, QApplication::UnicodeUTF8));
        _mirror_label_mirror->setText(QApplication::translate("MirrorManagerGUI", "Mirror:", 0, QApplication::UnicodeUTF8));
        _mirror_push_button_add_mirror->setText(QApplication::translate("MirrorManagerGUI", "Add Mirror", 0, QApplication::UnicodeUTF8));
        mirror_tab_widget->setTabText(mirror_tab_widget->indexOf(_mirror_tab_edit), QApplication::translate("MirrorManagerGUI", "Edit", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MirrorManagerGUI", "File", 0, QApplication::UnicodeUTF8));
        menuConfiguration->setTitle(QApplication::translate("MirrorManagerGUI", "Configuration", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MirrorManagerGUI: public Ui_MirrorManagerGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MIRRORMANAGERGUI_H
