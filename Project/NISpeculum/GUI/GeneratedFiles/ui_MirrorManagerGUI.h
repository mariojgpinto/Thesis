/********************************************************************************
** Form generated from reading UI file 'MirrorManagerGUI.ui'
**
** Created: Mon 16. Sep 22:44:08 2013
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
#include <QtGui/QPushButton>
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
    QSlider *_mirror_horizontal_slider_max;
    QSlider *_mirror_horizontal_slider_min;
    QFrame *line_4;
    QLabel *_mirror_label_min;
    QLabel *_mirror_label_max;
    QFrame *line_5;
    QPushButton *_mirror_push_button_add;
    QMenuBar *menubar;
    QMenu *menuFile;

    void setupUi(QMainWindow *MirrorManager)
    {
        if (MirrorManager->objectName().isEmpty())
            MirrorManager->setObjectName(QString::fromUtf8("MirrorManager"));
        MirrorManager->resize(540, 469);
        _mirror_action_close = new QAction(MirrorManager);
        _mirror_action_close->setObjectName(QString::fromUtf8("_mirror_action_close"));
        centralwidget = new QWidget(MirrorManager);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        _mirror_label_a = new QLabel(centralwidget);
        _mirror_label_a->setObjectName(QString::fromUtf8("_mirror_label_a"));
        _mirror_label_a->setGeometry(QRect(10, 330, 61, 16));
        _mirror_horizontal_slider_a = new QSlider(centralwidget);
        _mirror_horizontal_slider_a->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_a"));
        _mirror_horizontal_slider_a->setGeometry(QRect(100, 330, 431, 20));
        _mirror_horizontal_slider_a->setMinimum(-1000);
        _mirror_horizontal_slider_a->setMaximum(1000);
        _mirror_horizontal_slider_a->setOrientation(Qt::Horizontal);
        _mirror_label_b = new QLabel(centralwidget);
        _mirror_label_b->setObjectName(QString::fromUtf8("_mirror_label_b"));
        _mirror_label_b->setGeometry(QRect(10, 360, 61, 16));
        _mirror_horizontal_slider_b = new QSlider(centralwidget);
        _mirror_horizontal_slider_b->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_b"));
        _mirror_horizontal_slider_b->setGeometry(QRect(100, 360, 431, 20));
        _mirror_horizontal_slider_b->setMinimum(-1000);
        _mirror_horizontal_slider_b->setMaximum(1000);
        _mirror_horizontal_slider_b->setOrientation(Qt::Horizontal);
        _mirror_label_c = new QLabel(centralwidget);
        _mirror_label_c->setObjectName(QString::fromUtf8("_mirror_label_c"));
        _mirror_label_c->setGeometry(QRect(10, 390, 61, 16));
        _mirror_horizontal_slider_c = new QSlider(centralwidget);
        _mirror_horizontal_slider_c->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_c"));
        _mirror_horizontal_slider_c->setGeometry(QRect(100, 390, 431, 20));
        _mirror_horizontal_slider_c->setMinimum(-1000);
        _mirror_horizontal_slider_c->setMaximum(1000);
        _mirror_horizontal_slider_c->setOrientation(Qt::Horizontal);
        _mirror_label_d = new QLabel(centralwidget);
        _mirror_label_d->setObjectName(QString::fromUtf8("_mirror_label_d"));
        _mirror_label_d->setGeometry(QRect(10, 420, 61, 16));
        _mirror_horizontal_slider_d = new QSlider(centralwidget);
        _mirror_horizontal_slider_d->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_d"));
        _mirror_horizontal_slider_d->setGeometry(QRect(100, 420, 431, 20));
        _mirror_horizontal_slider_d->setMinimum(0);
        _mirror_horizontal_slider_d->setMaximum(1000);
        _mirror_horizontal_slider_d->setValue(500);
        _mirror_horizontal_slider_d->setOrientation(Qt::Horizontal);
        _mirror_widget = new QWidget(centralwidget);
        _mirror_widget->setObjectName(QString::fromUtf8("_mirror_widget"));
        _mirror_widget->setGeometry(QRect(290, 50, 240, 180));
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(80, 330, 20, 111));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        _mirror_spin_box = new QSpinBox(centralwidget);
        _mirror_spin_box->setObjectName(QString::fromUtf8("_mirror_spin_box"));
        _mirror_spin_box->setGeometry(QRect(70, 50, 91, 22));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 50, 51, 16));
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 300, 521, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralwidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(260, 50, 20, 181));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        _mirror_horizontal_slider_max = new QSlider(centralwidget);
        _mirror_horizontal_slider_max->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_max"));
        _mirror_horizontal_slider_max->setGeometry(QRect(100, 280, 431, 20));
        _mirror_horizontal_slider_max->setMinimum(400);
        _mirror_horizontal_slider_max->setMaximum(2000);
        _mirror_horizontal_slider_max->setValue(1000);
        _mirror_horizontal_slider_max->setOrientation(Qt::Horizontal);
        _mirror_horizontal_slider_min = new QSlider(centralwidget);
        _mirror_horizontal_slider_min->setObjectName(QString::fromUtf8("_mirror_horizontal_slider_min"));
        _mirror_horizontal_slider_min->setGeometry(QRect(100, 250, 431, 20));
        _mirror_horizontal_slider_min->setMinimum(400);
        _mirror_horizontal_slider_min->setMaximum(2000);
        _mirror_horizontal_slider_min->setValue(500);
        _mirror_horizontal_slider_min->setOrientation(Qt::Horizontal);
        line_4 = new QFrame(centralwidget);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setGeometry(QRect(10, 230, 521, 20));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        _mirror_label_min = new QLabel(centralwidget);
        _mirror_label_min->setObjectName(QString::fromUtf8("_mirror_label_min"));
        _mirror_label_min->setGeometry(QRect(10, 250, 81, 16));
        _mirror_label_max = new QLabel(centralwidget);
        _mirror_label_max->setObjectName(QString::fromUtf8("_mirror_label_max"));
        _mirror_label_max->setGeometry(QRect(10, 280, 81, 16));
        line_5 = new QFrame(centralwidget);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setGeometry(QRect(10, 30, 521, 20));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        _mirror_push_button_add = new QPushButton(centralwidget);
        _mirror_push_button_add->setObjectName(QString::fromUtf8("_mirror_push_button_add"));
        _mirror_push_button_add->setGeometry(QRect(10, 10, 75, 23));
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
        _mirror_label_min->setText(QApplication::translate("MirrorManager", "Min: 500mm", 0, QApplication::UnicodeUTF8));
        _mirror_label_max->setText(QApplication::translate("MirrorManager", "Max: 1000mm", 0, QApplication::UnicodeUTF8));
        _mirror_push_button_add->setText(QApplication::translate("MirrorManager", "Add Mirror", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MirrorManager", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MirrorManager: public Ui_MirrorManager {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MIRRORMANAGERGUI_H
