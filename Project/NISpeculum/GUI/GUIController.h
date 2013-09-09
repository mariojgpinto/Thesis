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
class PointSelectionGUI;

class __declspec(dllexport) GUIController{
	public:
		GUIController(Controller* controller);
		~GUIController();

		void hide_all();
	
		void run(int argc, char* argv[]);

		void update();

		void point_selection(int flag_id, cv::Mat* img = 0);

	private:
		SpeculumGUI *_speculum_gui;
		PointSelectionGUI *_point_selection;

		Controller* _controller;

		bool _point_selection_flag;
		int _point_selection_id;

		//QApplication *_q_application;//(argc, argv);
};

#endif