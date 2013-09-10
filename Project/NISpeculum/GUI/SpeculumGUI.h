#ifndef SPECULUM_H
#define SPECULUM_H

#include <QtGui/qmainwindow.h>
#include "ui_SpeculumGUI.h"

#include <ToolBoxQT.h>

class Controller;
class MirrorManagerGUI;

class __declspec(dllexport) SpeculumGUI : public QMainWindow
{
	Q_OBJECT

	public:
		SpeculumGUI(Controller* controller, MirrorManagerGUI *mirror_manager, QApplication* app, QWidget *parent = 0, Qt::WFlags flags = 0);
		~SpeculumGUI();

		void update_widget();

	public slots:
		void on_close();
		void on_save();
		void on_load();

		void on_pause();
		void on_button_add_floor();
		void on_button_add_mirror();
		void on_button_mirror_manager();

	protected:
		//Setup Methods
		void setup_windows();
		void setup_connections();

		void update_timer();

	private:
		Ui::SpeculumGUI* _ui;
		QApplication* _q_application;

		MirrorManagerGUI *_mirror_manager;

		Controller* _controller;

		ToolBoxQT::CVWidget * _cvwidget;
};

#endif // SPECULUM_H
