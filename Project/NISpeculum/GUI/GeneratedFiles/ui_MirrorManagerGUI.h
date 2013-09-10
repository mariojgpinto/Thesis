/********************************************************************************
** Form generated from reading UI file 'MirrorManagerGUI.ui'
**
** Created: Tue 10. Sep 17:35:56 2013
**      by: Qt User Interface Compiler version 4.8.2
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
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MirrorManager
{
public:
    QAction *_mirror_action_close;
    QWidget *centralwidget;
    QLabel *_mirror_label_a;
    QSlider *_mirror_horizontal_slider_a;
    QLabel *_mirror_label_b;
    QSlider *_mirror_horizontal_slider_b;
    QLabel *_mirror_label_c;
    QSlider *_mirror_horizontal_slider_c;
    QLabel *_mirror_label_d;
    QSlider *_mirror_horizontal_slider_d;
    QWidget *_mirror_widget;
    QFrame *line;
    QSpinBox *_mirror_spin_box;
    QLabel *label;
    QFrame *line_2;
    QFrame *line_3;
    QMenuBar *menubar;
    QMenu *menuFile;

    void setupUi(QMainWindow *MirrorManager)
    {
        if (MirrorManager->objectName().isEmpty())
            MirrorManager->setObjectName(QString::fromUtf8("MirrorManager"));
        MirrorManager->resize(540, 380);
        _mirror_action_close = new QAction(MirrorManager);
        _mirror_action_close->setObjectName(QString::fromUtf8("_mirror_action_close"));
        centralwidget = new QWidget(MirrorManager);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        _mirror_label_a = new QLabel(centralwidget);
        _mirror_label_a->setObjectName(QString::fromUtf8("_mirror_label_a"));
        _mirror_label_a->setGeometry(QRect(10, 230, 61, 16));
        _mirror_horizontal_slider_a = new QSlider(centralwidget);
        _mirror_horizontal_slider_a->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_a"));
        _mirror_horizontal_slider_a->setGeometry(QRect(100, 230, 431, 20));
        _mirror_horizontal_slider_a->setMinimum(-1000);
        _mirror_horizontal_slider_a->setMaximum(1000);
        _mirror_horizontal_slider_a->setOrientation(Qt::Horizontal);
        _mirror_label_b = new QLabel(centralwidget);
        _mirror_label_b->setObjectName(QString::fromUtf8("_mirror_label_b"));
        _mirror_label_b->setGeometry(QRect(10, 260, 61, 16));
        _mirror_horizontal_slider_b = new QSlider(centralwidget);
        _mirror_horizontal_slider_b->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_b"));
        _mirror_horizontal_slider_b->setGeometry(QRect(100, 260, 431, 20));
        _mirror_horizontal_slider_b->setMinimum(-1000);
        _mirror_horizontal_slider_b->setMaximum(1000);
        _mirror_horizontal_slider_b->setOrientation(Qt::Horizontal);
        _mirror_label_c = new QLabel(centralwidget);
        _mirror_label_c->setObjectName(QString::fromUtf8("_mirror_label_c"));
        _mirror_label_c->setGeometry(QRect(10, 290, 61, 16));
        _mirror_horizontal_slider_c = new QSlider(centralwidget);
        _mirror_horizontal_slider_c->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_c"));
        _mirror_horizontal_slider_c->setGeometry(QRect(100, 290, 431, 20));
        _mirror_horizontal_slider_c->setMinimum(-1000);
        _mirror_horizontal_slider_c->setMaximum(1000);
        _mirror_horizontal_slider_c->setOrientation(Qt::Horizontal);
        _mirror_label_d = new QLabel(centralwidget);
        _mirror_label_d->setObjectName(QString::fromUtf8("_mirror_label_d"));
        _mirror_label_d->setGeometry(QRect(10, 320, 61, 16));
        _mirror_horizontal_slider_d = new QSlider(centralwidget);
        _mirror_horizontal_slider_d->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_d"));
        _mirror_horizontal_slider_d->setGeometry(QRect(100, 320, 431, 20));
        _mirror_horizontal_slider_d->setMinimum(0);
        _mirror_horizontal_slider_d->setMaximum(1000);
        _mirror_horizontal_slider_d->setValue(500);
        _mirror_horizontal_slider_d->setOrientation(Qt::Horizontal);
        _mirror_widget = new QWidget(centralwidget);
        _mirror_widget->setObjectName(QString::fromUtf8("_mirror_widget"));
        _mirror_widget->setGeometry(QRect(290, 10, 240, 180));
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(80, 230, 20, 111));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        _mirror_spin_box = new QSpinBox(centralwidget);
        _mirror_spin_box->setObjectName(QString::fromUtf8("_mirror_spin_box"));
        _mirror_spin_box->setGeometry(QRect(70, 10, 91, 22));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 10, 51, 16));
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 200, 521, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralwidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(260, 10, 20, 181));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        MirrorManager->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MirrorManager);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 540, 18));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        MirrorManager->setMenuBar(menubar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(_mirror_action_close);

        retranslateUi(MirrorManager);

        QMetaObject::connectSlotsByName(MirrorManager);
    } // setupUi

    void retranslateUi(QMainWindow *MirrorManager)
    {
        MirrorManager->setWindowTitle(QApplication::translate("MirrorManager", "MainWindow", 0, QApplication::UnicodeUTF8));
        _mirror_action_close->setText(QApplication::translate("MirrorManager", "Close", 0, QApplication::UnicodeUTF8));
        _mirror_label_a->setText(QApplication::translate("MirrorManager", "A:", 0, QApplication::UnicodeUTF8));
        _mirror_label_b->setText(QApplication::translate("MirrorManager", "B:", 0, QApplication::UnicodeUTF8));
        _mirror_label_c->setText(QApplication::translate("MirrorManager", "C:", 0, QApplication::UnicodeUTF8));
        _mirror_label_d->setText(QApplication::translate("MirrorManager", "D:", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MirrorManager", "Mirror", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MirrorManager", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MirrorManager: public Ui_MirrorManager {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MIRRORMANAGERGUI_H
