#ifndef THREE_D_GUI_H
#define THREE_D_GUI_H

#include <QtGui/qmainwindow.h>
#include "ui_ThreeDGUI.h"

class Controller;

class __declspec(dllexport) ThreeDGUI : public QMainWindow
{
	Q_OBJECT

	public:
		ThreeDGUI(Controller* controller, QApplication* app, QWidget *parent = 0, Qt::WFlags flags = 0);
		~ThreeDGUI();

	public slots:
		void on_close();
		
		void on_check_box_polygon(int value);

		void on_button_save();

	protected:
		//Setup Methods
		void setup_connections();

	private:
		Ui::ThreeDGUI* _ui;
		QApplication* _q_application;

		Controller* _controller;
};

#endif//THREE_D_GUI_H
