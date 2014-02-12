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

		void on_spin_box_radius_normal(double value);

		void on_check_box_filter(int value);
		void on_combo_box_filter(QString value);


		void on_check_box_outliers(int value);

		void on_check_box_voxel(int value);
		void on_spin_box_voxel_x(double value);
		void on_spin_box_voxel_y(double value);
		void on_spin_box_voxel_z(double value);

		void on_combo_box_filter_3d(QString value);
		void on_spin_box_minneighbors(int value);
		void on_spin_box_radius(double value);
		void on_spin_box_meank(int value);
		void on_spin_box_stddev(double value);

		void on_spin_box_nframes(int value);

		void on_combo_box_file_type(QString value);
		void on_button_capture_model();

	protected:
		//Setup Methods
		void setup_connections();

	private:
		Ui::ThreeDGUI* _ui;
		QApplication* _q_application;

		Controller* _controller;
};

#endif//THREE_D_GUI_H
