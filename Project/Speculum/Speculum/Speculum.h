/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef SPECULUM_H
#define SPECULUM_H

#include <QtGui/QMainWindow>
#include "ui_Speculum.h"

#include <ntk/ntk.h>
#include "Preferences.h"
#include "MirrorManagerGUI.h"
#include "FloorManagerGUI.h"
//#include "Controller.h"

//#include <Arena.h>

class Speculum : public QMainWindow
{
	Q_OBJECT

public:
	Speculum(Controller *c, QApplication *a, QWidget *parent = 0, Qt::WFlags flags = 0);
	~Speculum();

public slots:
    //Update Slot Method
    void update_window(ntk::RGBDImage* image);

	//Actions
	//File
	void on_close();
	void on_source_file();
	void on_source_kinect();
	void on_save_model();

	//Configuration
	void on_manage_mirrors();
	void on_manage_floor();
	void on_save_configuration();
	void on_load_configuration();

	//Preferences
	void on_preferences_gui();
	
	//Buttons
	void on_button_floor();

protected:
    //Setup Methods
    void setup_windows();
    void setup_connections();

    //Processing Methods
    void process_image();
    void show_images();

private:
	Ui::SpeculumClass* ui;
	QApplication* app;
	
	Controller* _controller;

	bool _button_floor_flag;

	//Windows
	ntk::ImageWidget *_ntk_widget_left;
	ntk::ImageWidget *_ntk_widget_right1;
	ntk::ImageWidget *_ntk_widget_right2;

};

#endif // SPECULUM_H
