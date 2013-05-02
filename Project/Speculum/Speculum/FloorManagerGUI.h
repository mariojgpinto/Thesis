/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef FLOOR_MANAGER_GUI_H
#define FLOOR_MANAGER_GUI_H

#include <QtGui/QMainWindow>
#include "ui_FloorManagerGUI.h"

#include <ntk/ntk.h>
#include <ntk/gui/image_widget.h>
#include "controller.h"

#include <Arena.h>

class FloorManagerGUI : public QMainWindow
{
	Q_OBJECT
public:
	FloorManagerGUI(Controller *c, QApplication *a, QWidget *parent = 0, Qt::WFlags flags = 0);
	~FloorManagerGUI();

public slots:
	
	//Actions
	//File
	void on_close();

	//Buttons
	void on_set_area();
	void on_add_floor_normal();
	void on_save();

	void on_threshold_changed(double value);

	//Processing Methods
    void process_image();
    void show_images();

protected:
    //Setup Methods
    void setup_windows();
    void setup_connections();

private:
	Ui::FloorManagerGUI* ui;
	QApplication* app;
	
	Controller* _controller;

	bool _auto_update;

	//Windows
	ntk::ImageWidget *_ntk_widget_main;
	//ntk::ImageWidget *_ntk_widget_bottom_left;
	//ntk::ImageWidget *_ntk_widget_bottom_right;
};

#endif //FLOOR_MANAGER_GUI_H