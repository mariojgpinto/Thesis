#include "FloorManagerGUI.h"

#include "..\NISpeculum\Controller.h"
#include "..\NISpeculum\PropertyManager.h"
#include "..\NISpeculum\Floor.h"

#include "GUIController.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
FloorManagerGUI::FloorManagerGUI(Controller* controller, QApplication *app, QWidget *parent, Qt::WFlags flags): 
	QMainWindow(parent, flags),
	_q_application(app),
	_ui(new Ui::FloorManager)
{
	_ui->setupUi(this);

	this->_controller = controller;

	this->setup_windows();
	this->setup_connections();
}

FloorManagerGUI::~FloorManagerGUI()
{

}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void FloorManagerGUI::setup_windows(){
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

	this->_cvwidget = new ToolBoxQT::CVWidget(this->_ui->_floor_widget);
    this->_cvwidget->setObjectName(QString::fromUtf8("Color"));
    this->_cvwidget->setSizePolicy(sizePolicy1);
	this->_cvwidget->setFixedSize(this->_ui->_floor_widget->width(),this->_ui->_floor_widget->height());
}

void FloorManagerGUI::setup_connections(){
	this->_ui->_floor_action_close->setShortcut(Qt::Key_Escape);
	connect(this->_ui->_floor_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	this->_ui->_floor_push_button_add->setShortcut(QKeySequence("Ctrl+F"));
	connect(this->_ui->_floor_push_button_add,SIGNAL(clicked()),this,SLOT(on_button_add_floor()));

	connect(this->_ui->_floor_horizontal_slider_thresh, SIGNAL(valueChanged(int)), this, SLOT(on_slider_thresh(int)));

	connect(this->_ui->_floor_horizontal_slider_a, SIGNAL(valueChanged(int)), this, SLOT(on_slider_a(int)));
	connect(this->_ui->_floor_horizontal_slider_b, SIGNAL(valueChanged(int)), this, SLOT(on_slider_b(int)));
	connect(this->_ui->_floor_horizontal_slider_c, SIGNAL(valueChanged(int)), this, SLOT(on_slider_c(int)));
	connect(this->_ui->_floor_horizontal_slider_d, SIGNAL(valueChanged(int)), this, SLOT(on_slider_d(int)));

	connect(this->_ui->_floor_horizontal_slider_min, SIGNAL(valueChanged(int)), this, SLOT(on_slider_min(int)));
	connect(this->_ui->_floor_horizontal_slider_max, SIGNAL(valueChanged(int)), this, SLOT(on_slider_max(int)));
}

//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------
void FloorManagerGUI::on_close(){
	this->hide();
}

void FloorManagerGUI::on_button_add_floor(){
	static bool area = true;

	if(area){
		this->_controller->_gui->point_selection(1);
		area = false;
	}
	else{
		this->_controller->_gui->point_selection(2);
		area = true;
	}	
}

void FloorManagerGUI::on_slider_a(int value){
	ToolBox::Plane* plane = &this->_controller->_floor->_plane;

	this->_controller->_floor->_plane.set_normalized(value/1000.0,plane->_b,plane->_c,plane->_d);
	
	char buff[128];
	sprintf(buff,"A: %.4f\0",plane->_a);
	this->_ui->_floor_label_a->setText(buff);
	sprintf(buff,"B: %.4f\0",plane->_b);
	this->_ui->_floor_label_b->setText(buff);
	sprintf(buff,"C: %.4f\0",plane->_c);
	this->_ui->_floor_label_c->setText(buff);
	sprintf(buff,"D: %.4f\0",plane->_d);
	this->_ui->_floor_label_d->setText(buff);
}

void FloorManagerGUI::on_slider_b(int value){
	ToolBox::Plane* plane = &this->_controller->_floor->_plane;

	this->_controller->_floor->_plane.set_normalized(plane->_a,value/1000.0,plane->_c,plane->_d);
	
	char buff[128];
	sprintf(buff,"A: %.4f\0",plane->_a);
	this->_ui->_floor_label_a->setText(buff);
	sprintf(buff,"B: %.4f\0",plane->_b);
	this->_ui->_floor_label_b->setText(buff);
	sprintf(buff,"C: %.4f\0",plane->_c);
	this->_ui->_floor_label_c->setText(buff);
	sprintf(buff,"D: %.4f\0",plane->_d);
	this->_ui->_floor_label_d->setText(buff);
}

void FloorManagerGUI::on_slider_c(int value){
	ToolBox::Plane* plane = &this->_controller->_floor->_plane;

	this->_controller->_floor->_plane.set_normalized(plane->_a,plane->_b,value/1000.0,plane->_d);
	
	char buff[128];
	sprintf(buff,"A: %.4f\0",plane->_a);
	this->_ui->_floor_label_a->setText(buff);
	sprintf(buff,"B: %.4f\0",plane->_b);
	this->_ui->_floor_label_b->setText(buff);
	sprintf(buff,"C: %.4f\0",plane->_c);
	this->_ui->_floor_label_c->setText(buff);
	sprintf(buff,"D: %.4f\0",plane->_d);
	this->_ui->_floor_label_d->setText(buff);
}

void FloorManagerGUI::on_slider_d(int value){
	if(value < 0) return;
	ToolBox::Plane* plane = &this->_controller->_floor->_plane;

	this->_controller->_floor->_plane.set_normalized(plane->_a,plane->_b,plane->_c,value*-1);
	
	char buff[128];
	sprintf(buff,"A: %.4f\0",plane->_a);
	this->_ui->_floor_label_a->setText(buff);
	sprintf(buff,"B: %.4f\0",plane->_b);
	this->_ui->_floor_label_b->setText(buff);
	sprintf(buff,"C: %.4f\0",plane->_c);
	this->_ui->_floor_label_c->setText(buff);
	sprintf(buff,"D: %.4f\0",plane->_d);
	this->_ui->_floor_label_d->setText(buff);
}

void FloorManagerGUI::on_slider_thresh(int value){
	this->_controller->_floor->_thresh = value;
	
	char buff[128];
	sprintf(buff,"Thresh: %d\0",value);
	this->_ui->_floor_label_thresh->setText(buff);
}

void FloorManagerGUI::on_slider_min(int value){
	this->_controller->_floor->_depth_min = value;
	
	char buff[128];
	sprintf(buff,"Min: %dmm",value);
	this->_ui->_floor_label_min->setText(buff);
}

void FloorManagerGUI::on_slider_max(int value){
	this->_controller->_floor->_depth_max = value;
	
	char buff[128];
	sprintf(buff,"Max: %dmm",value);
	this->_ui->_floor_label_max->setText(buff);
}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
void FloorManagerGUI::update_values(){
	ToolBox::Plane* plane = &this->_controller->_floor->_plane;

	this->_ui->_floor_horizontal_slider_a->setValue(plane->_a * 1000);
	this->on_slider_a(plane->_a * 1000);
	this->_ui->_floor_horizontal_slider_b->setValue(plane->_b * 1000);
	this->on_slider_b(plane->_b * 1000);
	this->_ui->_floor_horizontal_slider_c->setValue(plane->_c * 1000);
	this->on_slider_c(plane->_c * 1000);
	this->_ui->_floor_horizontal_slider_d->setValue(plane->_d * -1);
	this->on_slider_d(plane->_d * 1000);

	this->_ui->_floor_horizontal_slider_thresh->setValue(this->_controller->_floor->_thresh);

	this->_ui->_floor_horizontal_slider_min->setValue(this->_controller->_floor->_depth_min);
	this->_ui->_floor_horizontal_slider_max->setValue(this->_controller->_floor->_depth_max);
}

void FloorManagerGUI::update_widget(){
	if(this->_controller->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE]){
		cv::Mat temp;
		this->_controller->_mat_color_bgr.copyTo(temp,this->_controller->_floor->_area_mask);
		this->_cvwidget->setImage(&temp);
	}
	else{
		this->_cvwidget->setImage(&this->_controller->_mat_color_bgr);
	}
}