/**
 * Author: Mario Pinto
 *
 * 
 */

#include "Speculum.h"

Speculum::Speculum(Controller *c, QApplication *a, QWidget *parent, Qt::WFlags flags):
	QMainWindow(parent, flags),
	app(a),
	_button_floor_flag(false),
	ui(new Ui::SpeculumClass)
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
// Setup
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
    connect(this->_controller->get_kinect(), SIGNAL(kinect_image(ntk::RGBDImage*)),this, SLOT(update_window(ntk::RGBDImage*)));

	this->ui->_main_action_exit->setShortcut(Qt::Key_Escape);
	connect(this->ui->_main_action_exit,SIGNAL(triggered()),this,SLOT(on_close()));

	this->ui->_main_action_preferences->setShortcut(QKeySequence("Ctrl+P"));
	connect(this->ui->_main_action_preferences,SIGNAL(triggered()),this,SLOT(on_preferences_gui()));

	this->ui->_main_action_add_mirror->setShortcut(QKeySequence("Ctrl+M"));
	connect(this->ui->_main_action_add_mirror,SIGNAL(triggered()),this,SLOT(on_add_mirror()));

	connect(this->ui->_main_push_button_floor, SIGNAL(clicked()), this, SLOT(on_button_floor()));
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
Mirror* mirror = new Mirror();
//-----------------------------------------------------------------------------
// SLOTS - CONFIGURATION ACTIONS 
//-----------------------------------------------------------------------------
namespace
{
	struct TrackerGroundMouseData
	{
		std::string window_name;
		cv::Mat3b image;
		std::vector<cv::Point*> points;
	};

	static void on_tracker_ground_mouse(int event, int x, int y, int flags, void *void_data)
	{
		if (event != CV_EVENT_LBUTTONUP)
			return;

		TrackerGroundMouseData* data = (TrackerGroundMouseData*)void_data;
		data->points.push_back(new cv::Point(x, y));
		circle(data->image, cv::Point(x,y), 5, cv::Scalar(255,255,255,255));
		imshow(data->window_name, data->image);
	}
}
void Speculum::on_add_mirror(){
	// Estimate ground plane equation asking 10 points to the user.
	TrackerGroundMouseData ground_mouse_data;
	ground_mouse_data.window_name = "Select the points to estimate the ground plane (left click). Press Space to exit";
	cv::namedWindow(ground_mouse_data.window_name);
	//		CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
	this->_controller->get_color_image()->copyTo(ground_mouse_data.image);
	cv::imshow(ground_mouse_data.window_name, ground_mouse_data.image);
	cv::setMouseCallback(ground_mouse_data.window_name, on_tracker_ground_mouse, &ground_mouse_data);
	while (cv::waitKey(30) != ' ')
	{

		QApplication::processEvents();
	}


	cv::destroyWindow(ground_mouse_data.window_name);

	if(ground_mouse_data.points.size() > 2){
		mirror->set_area(&ground_mouse_data.points);

		_button_floor_flag = true;
	}
}

//-----------------------------------------------------------------------------
// SLOTS - PREFERENCES ACTIONS 
//-----------------------------------------------------------------------------
void Speculum::on_preferences_gui(){
	this->_controller->get_preferences_window()->show(); 
}

void Speculum::on_button_floor(){
	
}

//-----------------------------------------------------------------------------
// Processing
//-----------------------------------------------------------------------------
void Speculum::process_image()
{
	this->_controller->process_images();
    //if(!this->_controller->get_user_window()->isHidden()){
    //    this->_controller->get_user_window()->process_image();
    //}
}

void Speculum::show_images()
{
	if(_button_floor_flag){
		cv::Mat3b aux;
		this->_controller->get_depth_as_color()->copyTo(aux,*mirror->get_mask());


		this->_ntk_widget_left->setImage(aux);
		this->_ntk_widget_right1->setImage(*this->_controller->get_color_image());
		this->_ntk_widget_right2->setImage(*this->_controller->get_depth_image());
	}
	else{

		this->_ntk_widget_left->setImage(*this->_controller->get_depth_as_color());
		this->_ntk_widget_right1->setImage(*this->_controller->get_color_image());
		this->_ntk_widget_right2->setImage(*this->_controller->get_depth_image());
	}
	this->_controller->show_images();
}
