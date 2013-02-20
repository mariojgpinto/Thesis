/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef MIRROR_MANAGER_GUI_H
#define MIRROR_MANAGER_GUI_H

#include <QtGui/QMainWindow>
#include "ui_MirrorManagerGUI.h"

#include <ntk/ntk.h>
#include <ntk/gui/image_widget.h>
#include "controller.h"

#include <Arena.h>

class MirrorManagerGUI : public QMainWindow
{
	Q_OBJECT
public:
	MirrorManagerGUI(Controller *c, QApplication *a, QWidget *parent = 0, Qt::WFlags flags = 0);
	~MirrorManagerGUI();

	//Setup
	void update_n_mirrors();

public slots:
	
	//Actions
	//File
	void on_close();

	//Edit
	void on_mirror_section(int value);
	void on_mirror_vertex(int value);
	void on_mirror_vertex_x(int value);
	void on_mirror_vertex_y(int value);

	//Processing Methods
    void process_image();
    void show_images();

	//Buttons
	void on_add_mirror();
	void on_calc_plane_by_points();

protected:
    //Setup Methods
    void setup_windows();
    void setup_connections();

private:
	Ui::MirrorManagerGUI *ui;
	QApplication* app;
	
	Controller* _controller;

	//Temporary Variables
	Arena* _arena;

	//Control Variables
	int _mirror_index;

	//Mat
	cv::Mat3b _mat_view_left;
	cv::Mat1b _mat_view_left_mask;
	cv::Mat3b _mat_top;
	cv::Mat3b _mat_bottom_left;
	cv::Mat3b _mat_bottom_right;

	//Windows
	ntk::ImageWidget *_ntk_widget_top;
	ntk::ImageWidget *_ntk_widget_bottom_left;
	ntk::ImageWidget *_ntk_widget_bottom_right;
	
	ntk::ImageWidget *_ntk_widget_view_left;
	ntk::ImageWidget *_ntk_widget_view1;
	ntk::ImageWidget *_ntk_widget_view2;
	ntk::ImageWidget *_ntk_widget_view3;
	ntk::ImageWidget *_ntk_widget_view4;
};

#endif //MIRROR_MANAGER_GUI_H
