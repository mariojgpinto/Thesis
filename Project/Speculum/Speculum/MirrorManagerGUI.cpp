/**
 * Author: Mario Pinto
 *
 * 
 */

#include "MirrorManagerGUI.h"

#include "ui_MirrorManagerGUI.h"
#include "controller.h"
#include "3DView.h"

#include <ntk/ntk.h>
#include <ntk/gui/image_widget.h>

#include <ntk/mesh/mesh_generator.h>

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
MirrorManagerGUI::MirrorManagerGUI(Controller *c, QApplication *a, QWidget *parent, Qt::WFlags flags):
	QMainWindow(parent, flags),
	app(a),
	ui(new Ui::MirrorManagerGUI),
	_mirror_index(0),
	_mat_view_left(480,640,cv::Vec3b(0,0,0)),
	_mat_view_left_mask(480,640,(const uchar)0),
	_mat_top(480,640,cv::Vec3b(0,0,0)),
	_mat_bottom_left(480,640,cv::Vec3b(0,0,0)),
	_mat_bottom_right(480,640,cv::Vec3b(0,0,0))
{
	ui->setupUi(this);

	this->_controller = c;
    this->_controller->set_mirror_manager_window(this);

	this->_controller->set_paused(true);

    this->setup_windows();
    this->setup_connections();

    this->_controller->set_paused(false);
}

MirrorManagerGUI::~MirrorManagerGUI()
{
	delete ui;
}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void MirrorManagerGUI::setup_windows(){
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

	this->_ntk_widget_top = new ntk::ImageWidget(this->ui->_mirror_widget_top);
    this->_ntk_widget_top->setObjectName(QString::fromUtf8("mirror_widget_top"));
    this->_ntk_widget_top->setSizePolicy(sizePolicy1);
	this->_ntk_widget_top->setFixedSize(this->ui->_mirror_widget_top->width(),
										this->ui->_mirror_widget_top->height());

	this->_ntk_widget_bottom_left = new ntk::ImageWidget(this->ui->_mirror_widget_bottom_left);
    this->_ntk_widget_bottom_left->setObjectName(QString::fromUtf8("mirror_widget_bottom_left"));
    this->_ntk_widget_bottom_left->setSizePolicy(sizePolicy1);
	this->_ntk_widget_bottom_left->setFixedSize(this->ui->_mirror_widget_bottom_left->width(),
												this->ui->_mirror_widget_bottom_left->height());

	this->_ntk_widget_bottom_right = new ntk::ImageWidget(this->ui->_mirror_widget_bottom_right);
    this->_ntk_widget_bottom_right->setObjectName(QString::fromUtf8("mirror_widget_bottom_right"));
    this->_ntk_widget_bottom_right->setSizePolicy(sizePolicy1);
	this->_ntk_widget_bottom_right->setFixedSize(	this->ui->_mirror_widget_bottom_right->width(),
													this->ui->_mirror_widget_bottom_right->height());

	this->_ntk_widget_view_left = new ntk::ImageWidget(this->ui->_mirror_view_widget_left);
    this->_ntk_widget_view_left->setObjectName(QString::fromUtf8("_mirror_view_widget_left"));
    this->_ntk_widget_view_left->setSizePolicy(sizePolicy1);
	this->_ntk_widget_view_left->setFixedSize(	this->ui->_mirror_view_widget_left->width(),
													this->ui->_mirror_view_widget_left->height());

	this->_ntk_widget_view1 = new ntk::ImageWidget(this->ui->_mirror_view_widget1);
    this->_ntk_widget_view1->setObjectName(QString::fromUtf8("_mirror_view_widget1"));
    this->_ntk_widget_view1->setSizePolicy(sizePolicy1);
	this->_ntk_widget_view1->setFixedSize(	this->ui->_mirror_view_widget1->width(),
													this->ui->_mirror_view_widget1->height());

	this->_ntk_widget_view2 = new ntk::ImageWidget(this->ui->_mirror_view_widget2);
    this->_ntk_widget_view2->setObjectName(QString::fromUtf8("_mirror_view_widget2"));
    this->_ntk_widget_view2->setSizePolicy(sizePolicy1);
	this->_ntk_widget_view2->setFixedSize(	this->ui->_mirror_view_widget2->width(),
													this->ui->_mirror_view_widget2->height());

	this->_ntk_widget_view3 = new ntk::ImageWidget(this->ui->_mirror_view_widget3);
    this->_ntk_widget_view3->setObjectName(QString::fromUtf8("_mirror_view_widget3"));
    this->_ntk_widget_view3->setSizePolicy(sizePolicy1);
	this->_ntk_widget_view3->setFixedSize(	this->ui->_mirror_view_widget3->width(),
													this->ui->_mirror_view_widget3->height());

	this->_ntk_widget_view4 = new ntk::ImageWidget(this->ui->_mirror_view_widget4);
    this->_ntk_widget_view4->setObjectName(QString::fromUtf8("_mirror_view_widget4"));
    this->_ntk_widget_view4->setSizePolicy(sizePolicy1);
	this->_ntk_widget_view4->setFixedSize(	this->ui->_mirror_view_widget4->width(),
													this->ui->_mirror_view_widget4->height());

	this->ui->_mirror_spinbox_mirror->setMaximum(0);
	this->ui->_mirror_spinbox_mirror->setMinimum(0);

    //int _min = (this->_controller->get_min_depth() - this->min_value) * 100 / this->max_value;
    //this->min_slider_change(_min);
    //this->ui->main_horizontalSlider_min_depth->setValue(_min);
    //int _max = (this->_controller->get_max_depth() - this->min_value) * 100 / this->max_value;
    //this->max_slider_change(_max);
    //this->ui->main_horizontalSlider_max_depth->setValue(_max);
}

void MirrorManagerGUI::setup_connections(){
	//Close Action and Shortcut
	this->ui->_mirror_action_close->setShortcut(Qt::Key_Escape);
	connect(this->ui->_mirror_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	//Add Mirror Action, Button and Shortcut
	this->ui->_mirror_action_add_mirror->setShortcut(QKeySequence("Ctrl+M"));
	connect(this->ui->_mirror_action_add_mirror,SIGNAL(triggered()),this,SLOT(on_add_mirror()));
	connect(this->ui->_mirror_push_button_add_mirror, SIGNAL(clicked()), this, SLOT(on_add_mirror()));
	
	connect(this->ui->_mirror_spinbox_mirror, SIGNAL(valueChanged(int)), this, SLOT(on_mirror_section(int)));

	connect(this->ui->_mirror_spinbox_vertix, SIGNAL(valueChanged(int)), this, SLOT(on_mirror_vertex(int)));
	connect(this->ui->_mirror_spinbox_vertix_x, SIGNAL(valueChanged(int)), this, SLOT(on_mirror_vertex_x(int)));
	connect(this->ui->_mirror_spinbox_vertix_y, SIGNAL(valueChanged(int)), this, SLOT(on_mirror_vertex_y(int)));
	
	connect(this->ui->_mirror_push_button_calc_points, SIGNAL(clicked()), this, SLOT(on_calc_plane_by_points()));
	//_mirror_push_button_calc_points


    //connect(this->ui->main_pushButton_background, SIGNAL(clicked()), this, SLOT(on_botton_background()));
    //connect(this->ui->main_pushButton_user, SIGNAL(clicked()), this, SLOT(on_botton_user()));
    //connect(this->ui->main_horizontalSlider_min_depth, SIGNAL(valueChanged(int)), this, SLOT(min_slider_change(int)));
    //connect(this->ui->main_horizontalSlider_max_depth, SIGNAL(valueChanged(int)), this, SLOT(max_slider_change(int)));
}

void MirrorManagerGUI::update_n_mirrors(){
	int coiso = this->_controller->get_arena()->get_n_mirrors();
	this->ui->_mirror_spinbox_mirror->setMaximum(coiso);
	this->ui->_mirror_spinbox_mirror->setMinimum(1);
	if(!this->ui->_mirror_spinbox_mirror->value() && coiso)
		this->ui->_mirror_spinbox_mirror->setValue(1);
	
	
}

//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SLOTS - FILE ACTIONS 
//-----------------------------------------------------------------------------
void MirrorManagerGUI::on_close(){
	this->hide();
}

//-----------------------------------------------------------------------------
// SLOTS - EDIT
//-----------------------------------------------------------------------------
void MirrorManagerGUI::on_mirror_section(int value){
	this->_mirror_index = value - 1;

	if(value){
		this->ui->_mirror_spinbox_vertix->setMinimum(0);
		this->ui->_mirror_spinbox_vertix->setMaximum(this->_arena->get_mirror(this->_mirror_index)->get_n_vertexes()-1);
		this->on_mirror_vertex(this->ui->_mirror_spinbox_vertix->value());
	}
}

int fuckthisshitX = 0;
int fuckthisshitY = 0;

void MirrorManagerGUI::on_mirror_vertex(int value){
	cv::Point *pt = this->_arena->get_mirror(this->_mirror_index)->get_vertex(value);

	if(pt){
		fuckthisshitX++;this->ui->_mirror_spinbox_vertix_x->setValue(pt->x);
		fuckthisshitY++;this->ui->_mirror_spinbox_vertix_y->setValue(pt->y);
	}
}

void MirrorManagerGUI::on_mirror_vertex_x(int value){
	cv::Point pt(this->ui->_mirror_spinbox_vertix_x->value(),this->ui->_mirror_spinbox_vertix_y->value());

	if(!fuckthisshitX)this->_arena->get_mirror(this->_mirror_index)->update_vertex(this->ui->_mirror_spinbox_vertix->value(),&pt);
	else fuckthisshitX--;
}

void MirrorManagerGUI::on_mirror_vertex_y(int value){
	cv::Point pt(this->ui->_mirror_spinbox_vertix_x->value(),this->ui->_mirror_spinbox_vertix_y->value());
	
	if(!fuckthisshitY)this->_arena->get_mirror(this->_mirror_index)->update_vertex(this->ui->_mirror_spinbox_vertix->value(),&pt);
	else fuckthisshitY--;
	
}

//-----------------------------------------------------------------------------
// SLOTS - ADD MIRROR
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
void MirrorManagerGUI::on_add_mirror(){
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
		Mirror* mirror = new Mirror();

		mirror->set_area(&ground_mouse_data.points);

		this->_controller->get_arena()->add_mirror(mirror);

		this->_controller->get_mirror_manager_window()->update_n_mirrors();
	}
}

//-----------------------------------------------------------------------------
// SLOTS - CALC PLANE BY POINTS
//-----------------------------------------------------------------------------
void MirrorManagerGUI::on_calc_plane_by_points(){
	if(this->_arena->get_n_mirrors() == 0) return ;

	TrackerGroundMouseData ground_mouse_data;
	ground_mouse_data.window_name = "Select the points to estimate the ground plane (left click). Press Space to exit";
	cv::namedWindow(ground_mouse_data.window_name);
	
	Mirror* mirror = this->_arena->get_mirror(this->_mirror_index);

	this->_controller->get_depth_as_color()->copyTo(ground_mouse_data.image,*mirror->get_area_mask());
	
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
		mirror->set_plane(plane->a,plane->b,plane->c,plane->d);
	}

	
	/*

	cv::Mat1b _floor_mask;
	cv::Size size = this->_controller->get_RGBDImage()->depth().size();

	_floor_mask.create(size);
	_floor_mask.ones(size);

	ntk::Mesh mesh;

	float dist = 0.0f;
	for(int j = 0 ; j < this->_controller->get_RGBDImage()->depth().rows ; j++){
		for(int i = 0 ; i < this->_controller->get_RGBDImage()->depth().cols ; i++){
			cv::Point p(i,j);
			cv::Point3f pf = this->_controller->get_RGBDImage()->calibration()->depth_pose->unprojectFromImage(p, this->_controller->get_RGBDImage()->depth()(p));

			dist = plane->distanceToPlane(pf);

			if(dist < 0.03){
				_floor_mask[j][i] = 0;
			}
			else{
				_floor_mask[j][i] = 255;
			}
		}
	}


	cv::Vec3f normal = plane->normal();
	normal*=-1;
	cv::Mat1f depth2;
	cv::Mat1f depth;
	this->_controller->get_depth_image()->copyTo(depth2,*this->_arena->get_mirror(this->_mirror_index)->get_area_mask());
	depth2.copyTo(depth,_floor_mask);

	for(int j = 0 ; j < depth.rows ; j++){
		for(int i = 0 ; i < depth.cols ; i++){
			cv::Point p(i,j);
			if(depth(p)){
				cv::Point3f pf = this->_controller->get_RGBDImage()->calibration()->depth_pose->unprojectFromImage(p, this->_controller->get_RGBDImage()->depth()(p));
				
				double dp = plane->distanceToPlane(pf);
				cv::Vec3b color = (*this->_controller->get_color_image())(p);

				ntk::Surfel surf;
				surf.location = cv::Point3f(pf);
				surf.color = cv::Vec3f::all(170);//cv::Vec3f(color.val[2],color.val[1],color.val[0]);//color;//cv::Vec3f::all(255);
				mesh.addPointFromSurfel(surf);

				ntk::Surfel surf2;

				cv::Point3f pt2;
				pt2.x = pf.x + (dp * 2 * normal.val[0]);
				pt2.y = pf.y + (dp * 2 * normal.val[1]);
				pt2.z = pf.z + (dp * 2 * normal.val[2]);

				

				surf2.location = cv::Point3f(pt2);
				surf2.color = cv::Vec3f(color.val[2],color.val[1],color.val[0]);//cv::Vec3f::all(255);
				mesh.addPointFromSurfel(surf2);
			}
		}
	}

	
	//this->_controller->get_3d_window()->add_mesh(mesh);













	TrackerGroundMouseData ground_mouse_data2;
	ground_mouse_data2.window_name = "Select the points to estimate the ground plane (left click). Press Space to exit";
	cv::namedWindow(ground_mouse_data2.window_name);
	
	this->_controller->get_depth_as_color()->copyTo(ground_mouse_data2.image,*this->_arena->get_mirror(this->_mirror_index+1)->get_area_mask());
	
	// Estimate ground plane equation asking 10 points to the user.
	//		CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
	cv::imshow(ground_mouse_data2.window_name, ground_mouse_data.image);
	cv::setMouseCallback(ground_mouse_data2.window_name, on_tracker_ground_mouse, &ground_mouse_data2);
	while (cv::waitKey(30) != ' ')
	{
		QApplication::processEvents();
	}

	cv::destroyWindow(ground_mouse_data2.window_name);
	
	//TODO Verify if Points are within the mask

	ntk::Plane* plane2 = Arena::extract_plane(&ground_mouse_data2.points,this->_controller->get_RGBDImage(), 5);
	
	cv::Mat1b _floor_mask2;
	cv::Size size2 = this->_controller->get_RGBDImage()->depth().size();

	_floor_mask2.create(size2);
	_floor_mask2.ones(size2);

	dist = 0.0f;
	for(int j = 0 ; j < this->_controller->get_RGBDImage()->depth().rows ; j++){
		for(int i = 0 ; i < this->_controller->get_RGBDImage()->depth().cols ; i++){
			cv::Point p(i,j);
			cv::Point3f pf = this->_controller->get_RGBDImage()->calibration()->depth_pose->unprojectFromImage(p, this->_controller->get_RGBDImage()->depth()(p));

			dist = plane2->distanceToPlane(pf);

			if(dist < 0.03){
				_floor_mask2[j][i] = 0;
				
				
				//this->_floor_mask_inv[j][i] = 1;
			}
			else{
				_floor_mask2[j][i] = 255;
			}
		}
	}


	cv::Vec3f normal2 = plane2->normal();

	normal2*=-1;

	cv::Mat1f depth4;
	cv::Mat1f depth3;
	this->_controller->get_depth_image()->copyTo(depth4,*this->_arena->get_mirror(this->_mirror_index+1)->get_area_mask());
	depth4.copyTo(depth3,_floor_mask2);

	for(int j = 0 ; j < depth.rows ; j++){
		for(int i = 0 ; i < depth.cols ; i++){
			cv::Point p(i,j);
			if(depth3(p)){
				cv::Point3f pf = this->_controller->get_RGBDImage()->calibration()->depth_pose->unprojectFromImage(p, this->_controller->get_RGBDImage()->depth()(p));
				
				double dp = plane2->distanceToPlane(pf);
				cv::Vec3b color = (*this->_controller->get_color_image())(p);

				ntk::Surfel surf;
				surf.location = cv::Point3f(pf);
				surf.color = cv::Vec3f::all(170);//color;cv::Vec3f(color.val[2],color.val[1],color.val[0]);
				mesh.addPointFromSurfel(surf);

				ntk::Surfel surf2;

				cv::Point3f pt2;
				pt2.x = pf.x + (dp * 2 * normal2.val[0]);
				pt2.y = pf.y + (dp * 2 * normal2.val[1]);
				pt2.z = pf.z + (dp * 2 * normal2.val[2]);

				

				surf2.location = cv::Point3f(pt2);
				surf2.color = cv::Vec3f(color.val[2],color.val[1],color.val[0]);//cv::Vec3f::all(255);
				mesh.addPointFromSurfel(surf2);
			}
		}
	}

	
	this->_controller->get_3d_window()->add_mesh(mesh);

	//cv::Mat3b aux;
	//cv::Mat3b aux2;
	//this->_controller->get_depth_as_color()->copyTo(aux,_floor_mask);
	//aux.copyTo(aux2,*this->_arena->get_mirror(this->_mirror_index)->get_area_mask());
	//cv::imshow("Mask",_floor_mask);
	//cv::imshow("masked",aux2);

	//cv::waitKey(11);

	//cv::imshow("Mask",_floor_mask);
	//cv::imshow("masked",aux2);

	//cv::waitKey(0);*/
}

//-----------------------------------------------------------------------------
// Processing
//-----------------------------------------------------------------------------
void MirrorManagerGUI::process_image()
{
	this->_arena = this->_controller->get_arena();
	
	if(!this->ui->_mirror_tab_edit->isHidden()){

		
	} else
	if(!this->ui->_mirror_tab_view->isHidden()){
		std::vector<Mirror*>* mirrors = this->_arena->get_mirrors();
		cv::Mat1b aux = cv::Mat1b::zeros(this->_mat_view_left_mask.size());

		if(mirrors){
			for(unsigned int i = 0 ; i < mirrors->size() ; i++){
				cv::bitwise_or(*mirrors->at(i)->get_area_mask(),aux,aux);
			}

			aux.assignTo(this->_mat_view_left_mask);
		}
	}

	//this->_controller->process_images();
    //if(!this->_controller->get_user_window()->isHidden()){
    //    this->_controller->get_user_window()->process_image();
    //}
}

void MirrorManagerGUI::show_images()
{
	if(!this->ui->_mirror_tab_edit->isHidden()){
		if(this->_arena->get_n_mirrors() == 0){
			this->_ntk_widget_top->setImage(*this->_controller->get_depth_as_color());
			this->_ntk_widget_bottom_left->setImage(*this->_controller->get_color_image());
			this->_ntk_widget_bottom_right->setImage(*this->_controller->get_depth_image());
		} 
		else{
			cv::Mat3b aux;
			this->_controller->get_depth_as_color()->copyTo(aux,*this->_arena->get_mirror(this->_mirror_index)->get_area_mask());

			cv::Mat3b aux2;
			this->_controller->get_color_image()->copyTo(aux2,*this->_arena->get_mirror(this->_mirror_index)->get_area_mask());

			this->_ntk_widget_top->setImage(aux);
			this->_ntk_widget_bottom_right->setImage(*this->_arena->get_mirror(this->_mirror_index)->get_area_mask());
			this->_ntk_widget_bottom_left->setImage(aux2);
		}
	} else													
	if(!this->ui->_mirror_tab_view->isHidden()){
		cv::Mat3b aux;
		this->_controller->get_depth_as_color()->copyTo(aux,_mat_view_left_mask);

		//_mat_view_left
		this->_ntk_widget_view_left->setImage(aux);

		if(this->_arena->get_mirror(0)){
			this->_ntk_widget_view1->setImage(*this->_arena->get_mirror(0)->get_area_mask());
		}
		else{
			this->_ntk_widget_view1->setImage(*this->_controller->get_depth_as_color());
		}

		if(this->_arena->get_mirror(1)){
			this->_ntk_widget_view2->setImage(*this->_arena->get_mirror(1)->get_area_mask());
		}
		else{
			this->_ntk_widget_view2->setImage(*this->_controller->get_depth_as_color());
		}

		if(this->_arena->get_mirror(2)){
			this->_ntk_widget_view3->setImage(*this->_arena->get_mirror(2)->get_area_mask());
		}
		else{
			this->_ntk_widget_view3->setImage(*this->_controller->get_depth_as_color());
		}

		if(this->_arena->get_mirror(3)){
			this->_ntk_widget_view4->setImage(*this->_arena->get_mirror(3)->get_area_mask());
		}
		else{
			this->_ntk_widget_view4->setImage(*this->_controller->get_depth_as_color());
		}
	}
}
