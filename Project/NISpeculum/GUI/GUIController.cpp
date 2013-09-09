/** 
 * @file	GUIController.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	July, 2013 
 * @brief	Implementation of the GUIController Class.
 *
 * @details	Detailed Information.
 */
#include "GUIController.h"

#include "SpeculumGUI.h"
#include "..\NISpeculum\Controller.h"
#include "..\NISpeculum\PropertyManager.h"
#include "..\NISpeculum\Mirror.h"

#include <PointSelectionGUI.h>

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------

/**
 *
 */
GUIController::GUIController(Controller* controller){
	this->_controller = controller;

	this->_point_selection_flag = false;
}

GUIController::~GUIController(){

}

//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void GUIController::hide_all(){

}
	
void GUIController::run(int argc, char* argv[]){
	QApplication* app = new QApplication(argc,argv);
	//cv::namedWindow("Color");
	//kinect = new QNIKinect("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");
	//kinect->get_kinect()->set_processing_flag(NIKinect::DEPTH_COLOR, true);
	_point_selection = new PointSelectionGUI();
	_speculum_gui = new SpeculumGUI(_controller,app);
	
	//boost::thread coiso(&MainGUI::update_cycle, this);

	_speculum_gui->show();
	_point_selection->hide();
	app->exec();
}

void GUIController::point_selection(int flag_id, cv::Mat* image){
	cv::Mat img;
	switch(flag_id){
		case 1: //FLOOR POINTS
			if(image && image->cols && image->rows){
				image->copyTo(img);
			}
			else{
				this->_controller->_mat_color_bgr.copyTo(img,this->_controller->_mat_depth8UC1);
			}
			this->_point_selection_id = flag_id;
			break;
		case 2:
			if(image && image->cols && image->rows){
				image->copyTo(img);
			}
			else{
				this->_controller->_mat_color_bgr.copyTo(img,this->_controller->_mat_depth8UC1);
			}
			this->_point_selection_id = flag_id;
			break;
		case 3:
			if(image && image->cols && image->rows){
				image->copyTo(img);
			}
			else{
				this->_controller->_mat_color_bgr.copyTo(img,this->_controller->_mirrors[this->_controller->_mirrors.size()-1]->_area_mask);
			}
			this->_point_selection_id = flag_id;
			break;
		default: return;
	}

	this->_point_selection->set_image(img);
	this->_point_selection_flag = true;
	this->_point_selection->show();
}

void GUIController::update(){
	this->_speculum_gui->update_widget();

	if(this->_point_selection_flag){
		if(this->_point_selection->isHidden()){
			this->_point_selection->get_points(*this->_controller->_aux_points);
			
			this->_controller->_property_manager->_flag_requests[PropertyManager::R_REQUEST] = true;

			switch(this->_point_selection_id){
				case 1: //FLOOR POINTS
					this->_controller->_property_manager->_flag_requests[PropertyManager::R_FLOOR] = true;
					break;
				case 2: //FLOOR POINTS
					this->_controller->_property_manager->_flag_requests[PropertyManager::R_MIRROR_AREA] = true;
					break;
				case 3: //FLOOR POINTS
					this->_controller->_property_manager->_flag_requests[PropertyManager::R_MIRROR_POINTS] = true;
					break;
			}
			
			this->_point_selection_flag = false;
		}
	}
}