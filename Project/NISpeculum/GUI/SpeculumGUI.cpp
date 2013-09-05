#include "SpeculumGUI.h"

#include "..\NISpeculum\Controller.h"
#include "..\NISpeculum\PropertyManager.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
SpeculumGUI::SpeculumGUI(Controller* controller, QApplication *app, QWidget *parent, Qt::WFlags flags): 
	QMainWindow(parent, flags),
	_q_application(app),
	_ui(new Ui::SpeculumGUI)
{
	_ui->setupUi(this);

	this->_controller = controller;

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

	this->_cvwidget = new ToolBoxQT::CVWidget(this->_ui->_widget);
    this->_cvwidget->setObjectName(QString::fromUtf8("Color"));
    this->_cvwidget->setSizePolicy(sizePolicy1);
	this->_cvwidget->setFixedSize(this->_ui->_widget->width(),this->_ui->_widget->height());


    //int _min = (this->_controller->get_min_depth() - this->min_value) * 100 / this->max_value;
    //this->min_slider_change(_min);
    //this->ui->main_horizontalSlider_min_depth->setValue(_min);
    //int _max = (this->_controller->get_max_depth() - this->min_value) * 100 / this->max_value;
    //this->max_slider_change(_max);
    //this->ui->main_horizontalSlider_max_depth->setValue(_max);
}

void SpeculumGUI::setup_connections(){
    //connect(this->_controller->get_kinect(), SIGNAL(kinect_image()),this, SLOT(update_window()));

	this->_ui->_action_close->setShortcut(Qt::Key_Escape);
	connect(this->_ui->_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	this->_ui->_action_save->setShortcut(QKeySequence("Ctrl+S"));
	connect(this->_ui->_action_save,SIGNAL(triggered()),this,SLOT(on_save()));

	this->_ui->_action_load->setShortcut(QKeySequence("Ctrl+L"));
	connect(this->_ui->_action_load,SIGNAL(triggered()),this,SLOT(on_load()));

	this->_ui->_main_pushButton_add_floor->setShortcut(QKeySequence("Ctrl+F"));
	connect(this->_ui->_main_pushButton_add_floor,SIGNAL(clicked()),this,SLOT(on_button_add_floor()));
		
	this->_ui->_main_pushButton_add_mirror->setShortcut(QKeySequence("Ctrl+M"));
	connect(this->_ui->_main_pushButton_add_mirror,SIGNAL(clicked()),this,SLOT(on_button_add_mirror()));

    //connect(this->ui->main_pushButton_floor, SIGNAL(clicked()), this, SLOT(on_botton_floor()));
    //connect(this->ui->main_pushButton_background, SIGNAL(clicked()), this, SLOT(on_botton_background()));
    //connect(this->ui->main_pushButton_user, SIGNAL(clicked()), this, SLOT(on_botton_user()));
    //connect(this->ui->main_horizontalSlider_min_depth, SIGNAL(valueChanged(int)), this, SLOT(min_slider_change(int)));
    //connect(this->ui->main_horizontalSlider_max_depth, SIGNAL(valueChanged(int)), this, SLOT(max_slider_change(int)));
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

void SpeculumGUI::on_button_add_floor(){
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_REQUEST] = true;
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_FLOOR] = true;

	cv::namedWindow("Add Floor");
}

void SpeculumGUI::on_button_add_mirror(){
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_REQUEST] = true;
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_MIRROR] = true;

	cv::namedWindow("Add Mirror");
	cv::namedWindow("win1");
}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
void SpeculumGUI::update_widget(){
	//if(img)
		//this->_cvwidget->setImage(img);
	if(this->_controller->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE]){
		//cv::Mat temp;

		//this->_controller->_mat_color_bgr.copyTo(temp,this->_controller->_mask_main);
		this->_cvwidget->setImage(&this->_controller->_mask_main);
	}
	else
		this->_cvwidget->setImage(&this->_controller->_mat_depth8UC1);
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