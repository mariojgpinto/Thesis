#include "SpeculumGUI.h"

#include "..\NISpeculum\Controller.h"
#include "..\NISpeculum\PropertyManager.h"
//#include "..\NISpeculum\Mirror.h"

#include "GUIController.h"
#include "MirrorManagerGUI.h"
#include "FloorManagerGUI.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
SpeculumGUI::SpeculumGUI(Controller* controller, MirrorManagerGUI *mirror_manager, FloorManagerGUI *floor_manager, QApplication *app, QWidget *parent, Qt::WFlags flags): 
	QMainWindow(parent, flags),
	_q_application(app),
	_ui(new Ui::SpeculumGUI)
{
	_ui->setupUi(this);

	this->_controller = controller;
	this->_mirror_manager = mirror_manager;
	this->_floor_manager = floor_manager;

	this->setup_windows();
	this->setup_connections();
}

SpeculumGUI::~SpeculumGUI()
{

}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void SpeculumGUI::setup_windows(){
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

	this->_cvwidget = new ToolBoxQT::CVWidget(this->_ui->_main_widget);
    this->_cvwidget->setObjectName(QString::fromUtf8("Color"));
    this->_cvwidget->setSizePolicy(sizePolicy1);
	this->_cvwidget->setFixedSize(this->_ui->_main_widget->width(),this->_ui->_main_widget->height());
}

void SpeculumGUI::setup_connections(){
	this->_ui->_action_close->setShortcut(Qt::Key_Escape);
	connect(this->_ui->_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	this->_ui->_action_save->setShortcut(QKeySequence("Ctrl+S"));
	connect(this->_ui->_action_save,SIGNAL(triggered()),this,SLOT(on_save()));

	this->_ui->_action_load->setShortcut(QKeySequence("Ctrl+L"));
	connect(this->_ui->_action_load,SIGNAL(triggered()),this,SLOT(on_load()));

	this->_ui->_main_push_button_pause->setShortcut(QKeySequence("Ctrl+P"));
	connect(this->_ui->_main_push_button_pause,SIGNAL(clicked()),this,SLOT(on_pause()));

	this->_ui->_main_push_button_floor_manager->setShortcut(QKeySequence("Ctrl+F"));
	connect(this->_ui->_main_push_button_floor_manager,SIGNAL(clicked()),this,SLOT(on_button_floor_manager()));
	this->_ui->_main_push_button_mirror_manager->setShortcut(QKeySequence("Ctrl+M"));
	connect(this->_ui->_main_push_button_mirror_manager,SIGNAL(clicked()),this,SLOT(on_button_mirror_manager()));
	
	
	connect(this->_ui->_main_horizontalSlider_min, SIGNAL(valueChanged(int)), this, SLOT(on_slider_min(int)));
	connect(this->_ui->_main_horizontalSlider_max, SIGNAL(valueChanged(int)), this, SLOT(on_slider_max(int)));
}

//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------
void SpeculumGUI::on_close(){
	this->_q_application->quit();
	exit(0);
}

void SpeculumGUI::on_save(){
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_REQUEST] = true;
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_SAVE] = true;
}

void SpeculumGUI::on_load(){
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_REQUEST] = true;
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_LOAD] = true;
}

void SpeculumGUI::on_pause(){
	this->_controller->_property_manager->_pause = !this->_controller->_property_manager->_pause;

	if(this->_controller->_property_manager->_pause)
		this->_ui->_main_push_button_pause->setText("Run");
	else
		this->_ui->_main_push_button_pause->setText("Pause");
}

void SpeculumGUI::on_button_mirror_manager(){
	this->_mirror_manager->update_values();
	this->_mirror_manager->show();
}

void SpeculumGUI::on_button_floor_manager(){
	this->_floor_manager->update_values();
	this->_floor_manager->show();
}

void SpeculumGUI::on_slider_min(int value){
	this->_controller->_property_manager->_depth_min = value;

	char buff[128];
	sprintf(buff,"Min: %dmm",this->_controller->_property_manager->_depth_min);
	this->_ui->_main_label_min->setText(buff);
}

void SpeculumGUI::on_slider_max(int value){
	this->_controller->_property_manager->_depth_max = value;

	char buff[128];
	sprintf(buff,"Max: %dmm",this->_controller->_property_manager->_depth_max);
	this->_ui->_main_label_max->setText(buff);
}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
void SpeculumGUI::update_widget(){
	//if(img)
		//this->_cvwidget->setImage(img);
	if(this->_controller->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE]){
		cv::Mat temp;

		this->_controller->_mat_color_bgr.copyTo(temp,this->_controller->_mask_main);
		this->_cvwidget->setImage(&temp);
	}
	else
		this->_cvwidget->setImage(&this->_controller->_mat_color_bgr);
}

//void SpeculumGUI::update_timer(){
//	++this->_frame_counter;
//    if (this->_frame_counter == 15)
//    {
//        double current_tick = cv::getTickCount();
//        this->_frame_rate = this->_frame_counter / ((current_tick - this->_last_tick)/cv::getTickFrequency());
//        this->_last_tick = current_tick;
//        this->_frame_counter = 0;
//    }
//
//	QString status = QString("FPS = %1").arg(this->_frame_rate, 0, 'f', 1);
//
//	this->setWindowTitle(status);
//}