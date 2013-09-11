/********************************************************************************
** Form generated from reading UI file '3DView.ui'
**
** Created: Wed 11. Sep 20:31:18 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_3DVIEW_H
#define UI_3DVIEW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>
#include <ntk/mesh/mesh_viewer.h>

QT_BEGIN_NAMESPACE

class Ui_View3D
{
public:
    QAction *view3d_action_close;
    QWidget *centralwidget;
    QFrame *frame;
    QHBoxLayout *horizontalLayout;
    QCheckBox *colorMappingCheckBox;
    QPushButton *saveMeshPushButton;
    QPushButton *pointCloudPushButton;
    QPushButton *surfelsPushButton;
    QPushButton *trianglePushButton;
    QSpacerItem *horizontalSpacer;
    QPushButton *resetCamera;
    QFrame *frame_2;
    QHBoxLayout *horizontalLayout_2;
    ntk::MeshViewer *mesh_view;
    QFrame *frame_3;
    QHBoxLayout *horizontalLayout_3;
    QSpinBox *_view3d_spinBox_nframes;
    QLabel *label;
    QPushButton *_view3d_pushButton_capture;
    QFrame *line;
    QPushButton *_view3d_pushButton_capture_single;
    QFrame *line_2;
    QSpacerItem *horizontalSpacer_2;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *View3D)
    {
        if (View3D->objectName().isEmpty())
            View3D->setObjectName(QString::fromUtf8("View3D"));
        View3D->resize(913, 665);
        view3d_action_close = new QAction(View3D);
        view3d_action_close->setObjectName(QString::fromUtf8("view3d_action_close"));
        centralwidget = new QWidget(View3D);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(10, 11, 895, 43));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        horizontalLayout = new QHBoxLayout(frame);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        colorMappingCheckBox = new QCheckBox(frame);
        colorMappingCheckBox->setObjectName(QString::fromUtf8("colorMappingCheckBox"));
        colorMappingCheckBox->setChecked(true);

        horizontalLayout->addWidget(colorMappingCheckBox);

        saveMeshPushButton = new QPushButton(frame);
        saveMeshPushButton->setObjectName(QString::fromUtf8("saveMeshPushButton"));

        horizontalLayout->addWidget(saveMeshPushButton);

        pointCloudPushButton = new QPushButton(frame);
        pointCloudPushButton->setObjectName(QString::fromUtf8("pointCloudPushButton"));

        horizontalLayout->addWidget(pointCloudPushButton);

        surfelsPushButton = new QPushButton(frame);
        surfelsPushButton->setObjectName(QString::fromUtf8("surfelsPushButton"));

        horizontalLayout->addWidget(surfelsPushButton);

        trianglePushButton = new QPushButton(frame);
        trianglePushButton->setObjectName(QString::fromUtf8("trianglePushButton"));

        horizontalLayout->addWidget(trianglePushButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        resetCamera = new QPushButton(frame);
        resetCamera->setObjectName(QString::fromUtf8("resetCamera"));

        horizontalLayout->addWidget(resetCamera);

        frame_2 = new QFrame(centralwidget);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(10, 60, 895, 492));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        horizontalLayout_2 = new QHBoxLayout(frame_2);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        mesh_view = new ntk::MeshViewer(frame_2);
        mesh_view->setObjectName(QString::fromUtf8("mesh_view"));

        horizontalLayout_2->addWidget(mesh_view);

        frame_3 = new QFrame(centralwidget);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        frame_3->setGeometry(QRect(10, 560, 895, 43));
        sizePolicy.setHeightForWidth(frame_3->sizePolicy().hasHeightForWidth());
        frame_3->setSizePolicy(sizePolicy);
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        horizontalLayout_3 = new QHBoxLayout(frame_3);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        _view3d_spinBox_nframes = new QSpinBox(frame_3);
        _view3d_spinBox_nframes->setObjectName(QString::fromUtf8("_view3d_spinBox_nframes"));
        _view3d_spinBox_nframes->setValue(10);

        horizontalLayout_3->addWidget(_view3d_spinBox_nframes);

        label = new QLabel(frame_3);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_3->addWidget(label);

        _view3d_pushButton_capture = new QPushButton(frame_3);
        _view3d_pushButton_capture->setObjectName(QString::fromUtf8("_view3d_pushButton_capture"));

        horizontalLayout_3->addWidget(_view3d_pushButton_capture);

        line = new QFrame(frame_3);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout_3->addWidget(line);

        _view3d_pushButton_capture_single = new QPushButton(frame_3);
        _view3d_pushButton_capture_single->setObjectName(QString::fromUtf8("_view3d_pushButton_capture_single"));

        horizontalLayout_3->addWidget(_view3d_pushButton_capture_single);

        line_2 = new QFrame(frame_3);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);

        horizontalLayout_3->addWidget(line_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);

        View3D->setCentralWidget(centralwidget);
        menubar = new QMenuBar(View3D);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 913, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        View3D->setMenuBar(menubar);
        statusbar = new QStatusBar(View3D);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        View3D->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(view3d_action_close);

        retranslateUi(View3D);

        QMetaObject::connectSlotsByName(View3D);
    } // setupUi

    void retranslateUi(QMainWindow *View3D)
    {
        View3D->setWindowTitle(QApplication::translate("View3D", "MainWindow", 0, QApplication::UnicodeUTF8));
        view3d_action_close->setText(QApplication::translate("View3D", "Close", 0, QApplication::UnicodeUTF8));
        colorMappingCheckBox->setText(QApplication::translate("View3D", "Color", 0, QApplication::UnicodeUTF8));
        saveMeshPushButton->setText(QApplication::translate("View3D", "SaveMesh", 0, QApplication::UnicodeUTF8));
        pointCloudPushButton->setText(QApplication::translate("View3D", "PointCloud", 0, QApplication::UnicodeUTF8));
        surfelsPushButton->setText(QApplication::translate("View3D", "Surfels", 0, QApplication::UnicodeUTF8));
        trianglePushButton->setText(QApplication::translate("View3D", "Triangles", 0, QApplication::UnicodeUTF8));
        resetCamera->setText(QApplication::translate("View3D", "Reset camera", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("View3D", "N Frames", 0, QApplication::UnicodeUTF8));
        _view3d_pushButton_capture->setText(QApplication::translate("View3D", "Capture Model", 0, QApplication::UnicodeUTF8));
        _view3d_pushButton_capture_single->setText(QApplication::translate("View3D", "Capture Single Frame", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("View3D", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class View3D: public Ui_View3D {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_3DVIEW_H
