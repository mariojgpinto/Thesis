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

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------

/**
 *
 */
GUIController::GUIController(Controller* controller){
	this->_controller = controller;
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
	_speculum_gui = new SpeculumGUI(_controller,app);
	
	//boost::thread coiso(&MainGUI::update_cycle, this);

	_speculum_gui->show();
	app->exec();
}

void GUIController::update(){
	this->_speculum_gui->update_widget();
}