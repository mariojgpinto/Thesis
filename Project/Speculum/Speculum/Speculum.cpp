/**
 * Author: Mario Pinto
 *
 * 
 */

#include "Speculum.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
Speculum::Speculum(Controller *c, QApplication *a, QWidget *parent, Qt::WFlags flags):
	QMainWindow(parent, flags),
	app(a),
	_button_floor_flag(false),
	ui(new Ui::SpeculumClass),
	_auto_update(false)
{
	ui->setupUi(this);

	this->_controller = c;
    this->_controller->set_viewer_window(this);

	this->_controller->set_paused(true);
    this->setup_windows();
    this->setup_connections();
    this->_controller->set_paused(false);
}

Speculum::~Speculum()
{
	delete ui;
}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void Speculum::setup_windows(){
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

	this->_ntk_widget_left = new ntk::ImageWidget(this->ui->_main_widget_left);
    this->_ntk_widget_left->setObjectName(QString::fromUtf8("DepthAsColor"));
    this->_ntk_widget_left->setSizePolicy(sizePolicy1);
	this->_ntk_widget_left->setFixedSize(this->ui->_main_widget_left->width(),this->ui->_main_widget_left->height());

	this->_ntk_widget_right1 = new ntk::ImageWidget(this->ui->_main_widget_right1);
    this->_ntk_widget_right1->setObjectName(QString::fromUtf8("DepthAsColor"));
    this->_ntk_widget_right1->setSizePolicy(sizePolicy1);
	this->_ntk_widget_right1->setFixedSize(this->ui->_main_widget_right1->width(),this->ui->_main_widget_right1->height());

	this->_ntk_widget_right2 = new ntk::ImageWidget(this->ui->_main_widget_right2);
    this->_ntk_widget_right2->setObjectName(QString::fromUtf8("DepthAsColor"));
    this->_ntk_widget_right2->setSizePolicy(sizePolicy1);
	this->_ntk_widget_right2->setFixedSize(this->ui->_main_widget_right2->width(),this->ui->_main_widget_right2->height());

    //int _min = (this->_controller->get_min_depth() - this->min_value) * 100 / this->max_value;
    //this->min_slider_change(_min);
    //this->ui->main_horizontalSlider_min_depth->setValue(_min);
    //int _max = (this->_controller->get_max_depth() - this->min_value) * 100 / this->max_value;
    //this->max_slider_change(_max);
    //this->ui->main_horizontalSlider_max_depth->setValue(_max);
}

void Speculum::setup_connections(){
	//Kinect Update
    connect(this->_controller->get_kinect(), SIGNAL(kinect_image(ntk::RGBDImage*)),this, SLOT(update_window(ntk::RGBDImage*)));

	//Menus and Action Shortcuts
	//Close
	this->ui->_main_action_exit->setShortcut(Qt::Key_Escape);
	connect(this->ui->_main_action_exit,SIGNAL(triggered()),this,SLOT(on_close()));
	
	//Preferences
	this->ui->_main_action_preferences->setShortcut(QKeySequence("Ctrl+P"));
	connect(this->ui->_main_action_preferences,SIGNAL(triggered()),this,SLOT(on_preferences_gui()));

	//Mirror Manager
	this->ui->_main_action_manage_mirrors->setShortcut(QKeySequence("Ctrl+M"));
	connect(this->ui->_main_action_manage_mirrors,SIGNAL(triggered()),this,SLOT(on_manage_mirrors()));

	//Floor Manager
	this->ui->_main_action_manager_floor->setShortcut(QKeySequence("Ctrl+F"));
	connect(this->ui->_main_action_manager_floor,SIGNAL(triggered()),this,SLOT(on_manage_floor()));

	//3D View
	this->ui->_main_action_3d_view->setShortcut(QKeySequence("Ctrl+3"));
	connect(this->ui->_main_action_3d_view,SIGNAL(triggered()),this,SLOT(on_3d_view()));

	//Save and Load Configuration
	this->ui->_main_action_save_configuration->setShortcut(QKeySequence("Ctrl+S"));
	connect(this->ui->_main_action_save_configuration,SIGNAL(triggered()),this,SLOT(on_save_configuration()));
	this->ui->_main_action_load_configuration->setShortcut(QKeySequence("Ctrl+L"));
	connect(this->ui->_main_action_load_configuration,SIGNAL(triggered()),this,SLOT(on_load_configuration()));

	//Choose Source
	connect(this->ui->_main_action_source_file,SIGNAL(triggered()),this,SLOT(on_source_file()));
	connect(this->ui->_main_action_source_kinect,SIGNAL(triggered()),this,SLOT(on_source_kinect()));

	//Save model
	connect(this->ui->_main_action_save_model,SIGNAL(triggered()),this,SLOT(on_save_model()));

	//Buttons 
	//Generate 3D 
	connect(this->ui->_main_push_button_generate_3d, SIGNAL(clicked()), this, SLOT(on_generate_3d_floor()));

	connect(this->ui->_main_push_button_auto_update, SIGNAL(clicked()), this, SLOT(on_auto_update()));
	

    //connect(this->ui->main_pushButton_background, SIGNAL(clicked()), this, SLOT(on_botton_background()));
    //connect(this->ui->main_pushButton_user, SIGNAL(clicked()), this, SLOT(on_botton_user()));
    //connect(this->ui->main_horizontalSlider_min_depth, SIGNAL(valueChanged(int)), this, SLOT(min_slider_change(int)));
    //connect(this->ui->main_horizontalSlider_max_depth, SIGNAL(valueChanged(int)), this, SLOT(max_slider_change(int)));
}


//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------
void Speculum::update_window(ntk::RGBDImage* image){
    this->_controller->get_kinect()->acquire_read_lock();

    this->_controller->update(image);

    this->process_image();
	
	this->show_images();

	this->_controller->get_kinect()->release_read_lock();
}


//-----------------------------------------------------------------------------
// SLOTS - FILE ACTIONS 
//-----------------------------------------------------------------------------
void Speculum::on_close(){
	this->app->quit();
	exit(0);
}

void Speculum::on_source_file(){
	//TODO Select File as Input Source
}

void Speculum::on_source_kinect(){
	//TODO Select Kinect as Input Source
}

void Speculum::on_save_model(){
	//TODO Save Model to File
}


//-----------------------------------------------------------------------------
// SLOTS - VIEW
//-----------------------------------------------------------------------------
void Speculum::on_3d_view(){
	this->_controller->get_3d_window()->show();
}


//-----------------------------------------------------------------------------
// SLOTS - CONFIGURATION ACTIONS 
//-----------------------------------------------------------------------------
void Speculum::on_manage_mirrors(){
	this->_controller->get_mirror_manager_window()->show();
	Sleep(500);
	if(this->_controller->get_arena()->get_mirrors()->size()){
		this->_controller->get_mirror_manager_window()->update_n_mirrors();
	}
}

void Speculum::on_manage_floor(){
	this->_controller->get_floor_manager_window()->show();
}

void Speculum::on_save_configuration(){
	if(this->_controller->get_arena()){
		this->_controller->get_arena()->save_to_file("..\\..\\Data\\config","arena");
	}
}

void Speculum::on_load_configuration(){
	//TODO Load Configurations
	if(this->_controller->get_arena()){
		this->_controller->get_arena()->load_from_file("..\\..\\Data\\config\\arena.xml");
	}
}


//-----------------------------------------------------------------------------
// SLOTS - PREFERENCES ACTIONS 
//-----------------------------------------------------------------------------
void Speculum::on_preferences_gui(){
	this->_controller->get_preferences_window()->show(); 
}


//-----------------------------------------------------------------------------
// SLOTS - BUTTONS
//-----------------------------------------------------------------------------
void Speculum::on_generate_3d_floor(){
	ntk::Mesh* mesh = this->_controller->get_arena()->calculate_mesh(this->_controller->get_RGBDImage());
	
	if(mesh){
		this->_controller->get_3d_window()->add_mesh(*mesh);
	}
}

void Speculum::on_auto_update(){
	this->_auto_update = !this->_auto_update;
}

//-----------------------------------------------------------------------------
// Processing
//-----------------------------------------------------------------------------
void Speculum::process_image()
{
	this->_controller->process_images();

	if(!this->_controller->get_preferences_window()->isHidden()){
		this->_controller->get_preferences_window()->process_image();
	}

	if(!this->_controller->get_mirror_manager_window()->isHidden()){
		this->_controller->get_mirror_manager_window()->process_image();
	}

	if(!this->_controller->get_floor_manager_window()->isHidden()){
		this->_controller->get_floor_manager_window()->process_image();
	}

	if(this->_controller->get_arena()->get_floor()){
		this->_controller->get_arena()->get_floor()->construct_floor_mask(this->_controller->get_RGBDImage());
	}

	if(this->_auto_update){
		if(this->_controller->get_arena()->get_floor()){
			this->_controller->get_arena()->get_floor()->construct_floor_mask(this->_controller->get_RGBDImage());
			
			ntk::Mesh *mesh = this->_controller->get_arena()->calculate_mesh(this->_controller->get_RGBDImage());

			if(mesh)
				this->_controller->get_3d_window()->add_mesh(*mesh);
		}
	}
}

void Speculum::show_images()
{
	if(!_controller->get_preferences_window()->isHidden()){
		_controller->get_preferences_window()->show_images();
	}

	if(!_controller->get_mirror_manager_window()->isHidden()){
		_controller->get_mirror_manager_window()->show_images();
	}

	cv::Mat3b aux;
	this->_controller->get_color_image()->copyTo(aux,*this->_controller->get_depth_mask());


	//this->_ntk_widget_left->setImage(*this->_controller->get_depth_as_color());
	this->_ntk_widget_left->setImage(aux);
	this->_ntk_widget_right1->setImage(*this->_controller->get_color_image());
	this->_ntk_widget_right2->setImage(*this->_controller->get_depth_image());

	//if(this->_button_floor_flag){
	//	cv::Mat3b aux;
	//	this->_controller->get_depth_as_color()->copyTo(aux,*this->_controller->get_arena()->get_mirror(0)->get_area_mask());


	//	this->_ntk_widget_left->setImage(aux);
	//	this->_ntk_widget_right1->setImage(*this->_controller->get_color_image());
	//	this->_ntk_widget_right2->setImage(*this->_controller->get_depth_image());
	//}
	//else{
	//	
	//	this->_ntk_widget_left->setImage(*this->_controller->get_depth_as_color());
	//	this->_ntk_widget_right1->setImage(*this->_controller->get_color_image());
	//	this->_ntk_widget_right2->setImage(*this->_controller->get_depth_image());
	//}
	this->_controller->show_images();
}