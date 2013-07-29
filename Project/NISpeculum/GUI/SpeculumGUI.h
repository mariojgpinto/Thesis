#ifndef SPECULUM_H
#define SPECULUM_H

#include <QtGui/qmainwindow.h>
#include "ui_SpeculumGUI.h"

#include <ToolBoxQT.h>

class Controller;

class __declspec(dllexport) SpeculumGUI : public QMainWindow
{
	Q_OBJECT

	public:
		SpeculumGUI(Controller* controller, QApplication* app, QWidget *parent = 0, Qt::WFlags flags = 0);
		~SpeculumGUI();

		void update_widget(cv::Mat *img);

	public slots:
		
		void on_close();

	protected:
		//Setup Methods
		void setup_windows();
		void setup_connections();

		void update_timer();

	private:
		Ui::SpeculumGUI* _ui;
		QApplication* _q_application;

		Controller* _controller;

		ToolBoxQT::CVWidget * _cvwidget;
};

#endif // SPECULUM_H
