#ifndef FLOOR_MANAGER_GUI_H
#define FLOOR_MANAGER_GUI_H

#include <QtGui/qmainwindow.h>
#include "ui_FloorManagerGUI.h"

#include <ToolBoxQT.h>

class Controller;

class __declspec(dllexport) FloorManagerGUI : public QMainWindow
{
	Q_OBJECT

	public:
		FloorManagerGUI(Controller* controller, QApplication* app, QWidget *parent = 0, Qt::WFlags flags = 0);
		~FloorManagerGUI();

		void update_widget();

		void update_values();

	public slots:
		void on_close();

		void on_button_add_floor();

		void on_floor_check_box_color(int value);
		void on_floor_spin_box_color_r(int value);
		void on_floor_spin_box_color_g(int value);
		void on_floor_spin_box_color_b(int value);

		void on_slider_a(int value);
		void on_slider_b(int value);
		void on_slider_c(int value);
		void on_slider_d(int value);

		void on_slider_thresh(int value);

		void on_slider_min(int value);
		void on_slider_max(int value);

	protected:
		//Setup Methods
		void setup_windows();
		void setup_connections();

	private:
		Ui::FloorManager* _ui;
		QApplication* _q_application;

		Controller* _controller;

		ToolBoxQT::CVWidget * _cvwidget;
};

#endif//FLOOR_MANAGER_GUI_H