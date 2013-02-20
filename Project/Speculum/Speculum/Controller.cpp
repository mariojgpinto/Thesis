/**
 * Author: Mario Pinto
 *
 * 
 */

#include "controller.h"
#include "Speculum.h"

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

    this->_kinect_image->depth().copyTo(*this->_depth_image);
    this->_kinect_image->rgb().copyTo(*this->_color_image);
    this->_kinect_image->depthMask().copyTo(*this->_depth_mask);

    ntk::compute_color_encoded_depth(this->_kinect_image->depth(), *this->_depth_as_color, &this->_min_depth, &this->_max_depth);
}

/*
 * Initialize the global variables
 */
void Controller::setup_variables(){
    this->_min_depth = 0.5;
    this->_max_depth = 1.0;
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
    }
}

/**
 *
 */
void Controller::process_images(){

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