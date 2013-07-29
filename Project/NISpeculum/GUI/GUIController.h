/** 
 * @file	GUIController.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	July, 2013 
 * @brief	Declaration of the GUIController Class.
 *
 * @details	Detailed Information.
 */
#ifndef _GUI_CONTROLLER
#define _GUI_CONTROLLER

#include <opencv2\opencv.hpp>

class SpeculumGUI;
class Controller;

class __declspec(dllexport) GUIController{
	public:
		GUIController(Controller* controller);
		~GUIController();

		void hide_all();
	
		void run(int argc, char* argv[]);

		void update(cv::Mat *img);

	private:
		SpeculumGUI *_speculum_gui;

		Controller* _controller;
		//QApplication *_q_application;//(argc, argv);
};

#endif