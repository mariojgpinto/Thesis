#ifndef MIRROR_MANAGER_GUI_H
#define MIRROR_MANAGER_GUI_H

#include <QtGui/qmainwindow.h>
#include "ui_MirrorManagerGUI.h"

#include <ToolBoxQT.h>

class Controller;

class __declspec(dllexport) MirrorManagerGUI : public QMainWindow
{
	Q_OBJECT

	public:
		MirrorManagerGUI(Controller* controller, QApplication* app, QWidget *parent = 0, Qt::WFlags flags = 0);
		~MirrorManagerGUI();

		void update_widget();

		void update_values();

	public slots:
		void on_close();

		void on_button_add_mirror();

		void on_mirror_check_box_color(int value);
		void on_mirror_spin_box_color_r(int value);
		void on_mirror_spin_box_color_g(int value);
		void on_mirror_spin_box_color_b(int value);

		void on_slider_a(int value);
		void on_slider_b(int value);
		void on_slider_c(int value);
		void on_slider_d(int value);

		void on_mirror_spin_box(int value);

		void on_slider_min(int value);
		void on_slider_max(int value);

	protected:
		//Setup Methods
		void setup_windows();
		void setup_connections();

	private:
		Ui::MirrorManager* _ui;
		QApplication* _q_application;

		Controller* _controller;

		ToolBoxQT::CVWidget * _cvwidget;

		int _idx_mirror;
};

#endif//MIRROR_MANAGER_GUI_H