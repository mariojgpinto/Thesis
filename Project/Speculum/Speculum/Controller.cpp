/**
 * Author: Mario Pinto
 *
 * 
 */

#include "controller.h"
#include "Speculum.h"
#include "3DView.h"
#include <Arena.h>

//------------------------------------------------
// CONSTRUCTORS
//------------------------------------------------
Controller::Controller(QKinect* k):	_last_tick(0),_frame_counter(0),_frame_rate(0){
    this->_kinect = k;

    this->_paused = false;

    this->setup_images();
    this->setup_variables();
}

Controller::~Controller(){

}

//------------------------------------------------
// SETUP
//------------------------------------------------

/*
 * Initialize the image variables
 */
void Controller::setup_images(){
    //Retrieves the first image from kinect for initializations
    this->_kinect_image = _kinect->get_image();

    //Initialize and create auxiliar images
    this->_depth_image = new cv::Mat1f(this->_kinect_image->depth().size());
    this->_color_image = new cv::Mat3b(this->_kinect_image->rgb().size());
    this->_depth_as_color = new cv::Mat3b(this->_kinect_image->rgb().size());
    this->_depth_mask = new cv::Mat1b(this->_kinect_image->depthMask().size());
    this->_diff = new cv::Mat1b(this->_kinect_image->depthMask().size());
	this->_buffer_mask = new cv::Mat1b(this->_kinect_image->depthMask().size());

    this->_kinect_image->depth().copyTo(*this->_depth_image);
    this->_kinect_image->rgb().copyTo(*this->_color_image);
    this->_kinect_image->depthMask().copyTo(*this->_depth_mask);
	this->_kinect_image->depthMask().copyTo(*this->_buffer_mask);

    ntk::compute_color_encoded_depth(this->_kinect_image->depth(), *this->_depth_as_color, &this->_min_depth, &this->_max_depth);
}

/*
 * Initialize the global variables
 */
void Controller::setup_variables(){
    this->_min_depth = 0.5;
    this->_max_depth = 1.0;

	this->_buffer_flag = false;
	this->_buffer = new std::vector<ntk::RGBDImage*>();
	this->_buffer_counter = 0;
	
}

//------------------------------------------------
// SETUP
//------------------------------------------------
/*
 *
 */
void Controller::update(ntk::RGBDImage* image){
    if(!this->_paused){
        //Retrieves the first image from kinect for initializations
        image->copyTo(*this->_kinect_image);

        //Copy the auxiliar images;
        this->_kinect_image->depth().copyTo(*this->_depth_image);
        this->_kinect_image->depthMask().copyTo(*this->_depth_mask);
        this->_kinect_image->rgb().copyTo(*this->_color_image);
        //this->_kinect_image->depthMask().copyTo(*this->_diff);

        ntk::compute_color_encoded_depth(*this->_depth_image, *this->_depth_as_color, &this->_min_depth, &this->_max_depth);

		this->update_timer();

		if(this->_buffer_flag){
			ntk::RGBDImage* bi = new ntk::RGBDImage();
			image->copyTo(*bi);
			this->_buffer->push_back(bi);

			if(this->_buffer->size() >= this->_buffer_counter){
				this->join_buffer();
				this->_buffer_flag = false;
			}
		}
	}
}

void Controller::join_buffer(){
	cv::Mat1i freq_n(this->_kinect_image->depth().size(),0);
	cv::Mat1f freq_v(this->_kinect_image->depth().size(),0);

	for(int bn = 0 ; bn < this->_buffer->size() ; bn++){
		cv::Mat1b mask = this->_buffer->at(bn)->depthMask();
		cv::Mat1f depth = this->_buffer->at(bn)->depth();

		for(int i = 0 ; i < mask.cols ; i++){
			for(int j = 0 ; j < mask.rows ; j++){
				cv::Point pt(i,j);

				if((*this->_buffer_mask)(pt) && mask(pt) && depth(pt) < 1){
					freq_n(pt)++;
					freq_v(pt)+=depth(pt);
				}
			}
		}
	}

	for(int i = 0 ; i < freq_n.cols ; i++){
		for(int j = 0 ; j < freq_n.rows ; j++){
			cv::Point pt(i,j);

			if(freq_n(pt)){
				freq_v(pt) /= (freq_n(pt));
			}
		}
	}
	
	//for(int i = 0 ; i < this->_buffer->size() ; i++){
	//	cv::imshow("coiso1", freq_v);
	//	cv::imshow("coiso2", freq_n);
	//	cv::imshow("coiso3", this->_buffer->at(i)->depth());
	//	cv::waitKey(0);
	//}

	cv::imshow("coiso1", freq_v);
	

	//cv::threshold(freq_n,freq_n,0.6,255.0,CV_THRESH_BINARY);

	cv::imshow("coiso2", freq_n);

	cv::waitKey(0);
	cv::morphologyEx(freq_v,freq_v,cv::MORPH_OPEN,cv::Mat(3,4,CV_8U,cv::Scalar(1)));
	//cv::Mat show;
	//cv::blur(freq_v,show,cv::Size(3,3));

	cv::imshow("coiso1", freq_v);
	cv::waitKey(0);

	cv::imwrite("capture_v.bmp",freq_v);
	cv::imwrite("capture_n.bmp",freq_n);

	ntk::RGBDImage* bi = new ntk::RGBDImage();
	this->_kinect_image->copyTo(*bi);
	freq_v.copyTo(bi->depth());
	freq_n.copyTo(bi->depthMask());

	ntk::Mesh *mesh = new ntk::Mesh();
	//ntk::Mesh *mesh = this->_arena->calculate_mesh(bi);

		//image max and min to 
	int max_width;
	int max_height;
	int min_width;
	int min_height;

		max_width = bi->depth().cols;
		max_height = bi->depth().rows;
		min_width = 0;
		min_height = 0;

	cv::Mat1f aux_depth = bi->depth();
	cv::Mat1b aux_depth_mask = bi->depthMask();
	cv::Mat3b aux_color = bi->rgb();
	ntk::Pose3D* aux_pose = bi->calibration()->depth_pose;
	ntk::Pose3D* aux_pose_rgb = bi->calibration()->rgb_pose;

	//Create a mask with all existing masks
	cv::Mat1b mega_mask;
	bi->depthMask().copyTo(mega_mask);


	mesh->clear();
	bi->rgb().copyTo(mesh->texture);   
	mesh->vertices.reserve(aux_depth.cols*aux_depth.rows);
    mesh->texcoords.reserve(aux_depth.cols*aux_depth.rows);
    mesh->colors.reserve(aux_depth.cols*aux_depth.rows);
	cv::Mat1i vertice_map(aux_depth.size());
	vertice_map = -1;

	for(int i = min_width ; i < max_width ; i++){
		for(int j = min_height ; j < max_height ; j++){
			cv::Point p(i,j);
			//If the point is part of the mask
			if(mega_mask(p)){
				//Unprojected point
				cv::Point3f pf = aux_pose->unprojectFromImage(p, aux_depth(p));
				
				//Color vector to pass from RGB to BGR
				cv::Vec3b color = aux_color(p);
				
				//Create the Surfel, add porperties and add to mesh
				ntk::Surfel surf;
				surf.location = cv::Point3f(pf);
				surf.color = cv::Vec3f(color.val[2],color.val[1],color.val[0]);
				mesh->addPointFromSurfel(surf);
				
				//cv::Point3f p2d_rgb = aux_pose_rgb->projectToImage(pf);
				//cv::Point2f texcoords = cv::Point2f(p2d_rgb.x/aux_color.cols, p2d_rgb.y/aux_color.rows);
				//vertice_map(j,i) = mesh->vertices.size();
				//mesh->vertices.push_back(pf);
				//mesh->colors.push_back(cv::Vec3f(color.val[2],color.val[1],color.val[0]));
				//mesh->texcoords.push_back(texcoords);
			}
		}
	}



	if(mesh)
		this->_view3d_window->add_mesh(*mesh);
}

/**
 *
 */

void Controller::process_images(){
	//m_mesh_generator->generate(*this->_kinect_image);
	//m_view3d_window->ui->mesh_view->addMesh(m_mesh_generator->mesh(), ntk::Pose3D(), ntk::MeshViewer::FLAT);
	//m_view3d_window->ui->mesh_view->swapScene();

	//ntk::Mesh meshs;
	//ntk::Surfel surf;
	//
	//meshs.addPointFromSurfel(surf);
}

void Controller::start_buffering(int n_frames){
	this->_buffer_counter = n_frames;
	this->_buffer->clear();
	this->_buffer_flag = true;

	if(this->_arena->get_floor() || (this->_arena->get_mirrors() && this->_arena->get_mirrors()->size()))
		this->_buffer_mask = new cv::Mat1b(this->_kinect_image->depth().size(),0);
	else
		this->_buffer_mask = new cv::Mat1b(this->_kinect_image->depth().size(),255);

	if(this->_arena->get_floor()){
		if(this->_arena->get_floor()->get_floor_mask())
			cv::bitwise_or(*this->_buffer_mask,*this->_arena->get_floor()->get_floor_mask(),*this->_buffer_mask);
	}

	if(this->_arena->get_mirrors()){
		std::vector<Mirror*>* vec = this->_arena->get_mirrors();
		if(vec){
			for(int i = 0 ; i < vec->size() ; i++){
				if(vec->at(i)->get_area_mask())
					cv::bitwise_or(*this->_buffer_mask,*vec->at(i)->get_area_mask(),*this->_buffer_mask);
			}
		}
	}
}

void Controller::update_timer(){
	++this->_frame_counter;
    if (this->_frame_counter == 5)
    {
        double current_tick = cv::getTickCount();
        this->_frame_rate = this->_frame_counter / ((current_tick - this->_last_tick)/cv::getTickFrequency());
        this->_last_tick = current_tick;
        this->_frame_counter = 0;
    }

	QString status = QString("FPS = %1").arg(this->_frame_rate, 0, 'f', 1);

	this->_main_window->setWindowTitle(status);
}

/**
 *
 */
void Controller::show_images(){

}