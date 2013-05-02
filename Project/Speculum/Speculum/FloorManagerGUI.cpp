/**
 * Author: Mario Pinto
 *
 * 
 */

#include "FloorManagerGUI.h"
#include "3DView.h"
//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
FloorManagerGUI::FloorManagerGUI(Controller *c, QApplication *a, QWidget *parent, Qt::WFlags flags):
	QMainWindow(parent, flags),
	app(a),
	ui(new Ui::FloorManagerGUI),
	_auto_update(false)
{
	ui->setupUi(this);

	this->_controller = c;
    this->_controller->set_floor_manager_window(this);

	this->_controller->set_paused(true);

    this->setup_windows();
    this->setup_connections();

    this->_controller->set_paused(false);
}

FloorManagerGUI::~FloorManagerGUI()
{
	delete ui;
}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void FloorManagerGUI::setup_windows(){
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

	this->_ntk_widget_main = new ntk::ImageWidget(this->ui->_floor_widget);
    this->_ntk_widget_main->setObjectName(QString::fromUtf8("floor_widget_main"));
    this->_ntk_widget_main->setSizePolicy(sizePolicy1);
	this->_ntk_widget_main->setFixedSize(this->ui->_floor_widget->width(),
										 this->ui->_floor_widget->height());

	//this->_ntk_widget_bottom_left = new ntk::ImageWidget(this->ui->_mirror_widget_bottom_left);
 //   this->_ntk_widget_bottom_left->setObjectName(QString::fromUtf8("mirror_widget_bottom_left"));
 //   this->_ntk_widget_bottom_left->setSizePolicy(sizePolicy1);
	//this->_ntk_widget_bottom_left->setFixedSize(this->ui->_mirror_widget_bottom_left->width(),
	//											this->ui->_mirror_widget_bottom_left->height());

	//this->_ntk_widget_bottom_right = new ntk::ImageWidget(this->ui->_mirror_widget_bottom_right);
 //   this->_ntk_widget_bottom_right->setObjectName(QString::fromUtf8("mirror_widget_bottom_right"));
 //   this->_ntk_widget_bottom_right->setSizePolicy(sizePolicy1);
	//this->_ntk_widget_bottom_right->setFixedSize(	this->ui->_mirror_widget_bottom_right->width(),
	//												this->ui->_mirror_widget_bottom_right->height());

    //int _min = (this->_controller->get_min_depth() - this->min_value) * 100 / this->max_value;
    //this->min_slider_change(_min);
    //this->ui->main_horizontalSlider_min_depth->setValue(_min);
    //int _max = (this->_controller->get_max_depth() - this->min_value) * 100 / this->max_value;
    //this->max_slider_change(_max);
    //this->ui->main_horizontalSlider_max_depth->setValue(_max);
}

void FloorManagerGUI::setup_connections(){
	//Close Action and Shortcut
 	this->ui->_floor_action_close->setShortcut(Qt::Key_Escape);
	connect(this->ui->_floor_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	//Save Action, Button and Shortcut
	this->ui->_floor_action_save->setShortcut(QKeySequence("Ctrl+M"));
	connect(this->ui->_floor_action_save,SIGNAL(triggered()),this,SLOT(on_save()));
	connect(this->ui->_floor_push_button_save, SIGNAL(clicked()), this, SLOT(on_save()));
	
	connect(this->ui->_floor_push_button_set_area, SIGNAL(clicked()), this, SLOT(on_set_area()));

	connect(this->ui->_floor_push_button_add_floor_normal, SIGNAL(clicked()), this, SLOT(on_add_floor_normal()));
    
	connect(this->ui->_floor_spin_box_double_thresh, SIGNAL(valueChanged(double)), this, SLOT(on_threshold_changed(double)));

	//connect(this->ui->main_pushButton_background, SIGNAL(clicked()), this, SLOT(on_botton_background()));
    //connect(this->ui->main_pushButton_user, SIGNAL(clicked()), this, SLOT(on_botton_user()));
    //connect(this->ui->main_horizontalSlider_min_depth, SIGNAL(valueChanged(int)), this, SLOT(min_slider_change(int)));
    //connect(this->ui->main_horizontalSlider_max_depth, SIGNAL(valueChanged(int)), this, SLOT(max_slider_change(int)));
}

//-----------------------------------------------------------------------------
// SLOTS
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

//-----------------------------------------------------------------------------
// SLOTS - BUTTONS
//-----------------------------------------------------------------------------
void FloorManagerGUI::on_set_area(){
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
		Floor* floor = NULL;

		if(!(floor = this->_controller->get_arena()->get_floor())){
			floor = new Floor();
		}

		floor->set_area(&ground_mouse_data.points);

		this->_controller->get_arena()->add_floor(floor);

		if(floor->get_area_mask()){
			cv::Mat3b aux;

			this->_controller->get_color_image()->copyTo(aux,*floor->get_area_mask());

			this->_ntk_widget_main->setImage(aux);
		}
	}
}

void FloorManagerGUI::on_add_floor_normal(){
	Floor* floor = NULL;

	if(!(floor = this->_controller->get_arena()->get_floor())){
		//Floor not created yet
		floor = new Floor();
		this->_controller->get_arena()->add_floor(floor);
	}

	TrackerGroundMouseData ground_mouse_data;
	ground_mouse_data.window_name = "Select the points to estimate the ground plane (left click). Press Space to exit";
	cv::namedWindow(ground_mouse_data.window_name);

	if(floor->check_flag(Floor::AREA))
		this->_controller->get_depth_as_color()->copyTo(ground_mouse_data.image,*floor->get_area_mask());
	else
		this->_controller->get_depth_as_color()->copyTo(ground_mouse_data.image);

	// Estimate ground plane equation asking 10 points to the user.
	//		CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
	cv::imshow(ground_mouse_data.window_name, ground_mouse_data.image);
	cv::setMouseCallback(ground_mouse_data.window_name, on_tracker_ground_mouse, &ground_mouse_data);
	while (cv::waitKey(30) != ' ')
	{
		QApplication::processEvents();
	}

	cv::destroyWindow(ground_mouse_data.window_name);
	
	//TODO Verify if Points are within the mask
	ntk::Plane* plane = Arena::extract_plane(&ground_mouse_data.points,this->_controller->get_RGBDImage(), 5);
	
	if(plane){
		floor->set_plane(plane->a,plane->b,plane->c,plane->d);
	}

	this->on_threshold_changed(1);
}

void FloorManagerGUI::on_threshold_changed(double value){
	double val = this->ui->_floor_spin_box_double_thresh->value();

	Floor* floor = this->_controller->get_arena()->get_floor();

	if(!floor) return ;

	floor->set_threshold(val);
	floor->construct_floor_mask(this->_controller->get_RGBDImage());

	if(floor->get_floor_mask()){
		cv::Mat3b aux;

		this->_controller->get_color_image()->copyTo(aux,*floor->get_floor_mask());

		this->_ntk_widget_main->setImage(aux);
	}

}

void FloorManagerGUI::on_save(){
	Floor* floor = this->_controller->get_arena()->get_floor();

	ntk::Mesh mesh;

	floor->add_perspective_to_mesh(&mesh,this->_controller->get_RGBDImage());

	this->_controller->get_3d_window()->add_mesh(mesh);
}

//-----------------------------------------------------------------------------
// SLOTS - FILE ACTIONS 
//-----------------------------------------------------------------------------
void FloorManagerGUI::on_close(){
	this->hide();
}


//-----------------------------------------------------------------------------
// Processing
//-----------------------------------------------------------------------------
void FloorManagerGUI::process_image()
{
	//this->_controller->process_images();
    //if(!this->_controller->get_user_window()->isHidden()){
    //    this->_controller->get_user_window()->process_image();
    //}
}

void FloorManagerGUI::show_images()
{
	Floor* floor = this->_controller->get_arena()->get_floor();
	if(floor){
		if(floor->check_flag(Floor::AREA) || floor->check_flag(Floor::FLOOR_MASK)){
			cv::Mat3b aux;

			if(floor->check_flag(Floor::FLOOR_MASK)){
				this->_controller->get_color_image()->copyTo(aux,*floor->get_floor_mask());
			}
			else{
				this->_controller->get_color_image()->copyTo(aux,*floor->get_area_mask());
			}
			this->_ntk_widget_main->setImage(aux);
		}
	}
	//this->_ntk_widget_top->setImage(*this->_controller->get_depth_as_color());
	//this->_ntk_widget_bottom_left->setImage(*this->_controller->get_color_image());
	//this->_ntk_widget_bottom_right->setImage(*this->_controller->get_depth_image());
}
