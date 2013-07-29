/** 
 * @file	Controller.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	July, 2013 
 * @brief	Declaration of the Controller Class.
 *
 * @details	Deailed Information.
 */
#ifndef _CONTROLLER
#define _CONTROLLER

class PropertyManager;
class GUIController;
class NIKinect;
class Viewer3D;
class Arena;

#include <opencv2\opencv.hpp>
#include <XnCppWrapper.h>
#include <boost\thread.hpp>
#include <pcl\point_types.h>
//#include "..\Arena\Arena.h"

class Controller{
	public:
		enum THREAD{
			KINECT,
			GUI,
			PCL_PRODUCER,
			PCL_CONSUMER
		};
		static const int _n_thread_flags = 4;


	public:
		Controller();
		~Controller();

		//Setup
		void init(const char* oni_path = 0);

		//Running
		void run(int argc, char* argv[]);

	private:
		//Thread Functions
		void thread_kinect();
		void thread_gui(int argc, char* argv[]);
		void thread_pcl_producer();
		void thread_pcl_consumer();

	public:
		//MEMBERS
		PropertyManager* _property_manager;
		GUIController* _gui;
		NIKinect* _kinect;
		Viewer3D* _3d_viewer;
		Arena* _arena;

		//VARIABLES
		cv::Mat _mat_color_bgr;
		cv::Mat _mat_color_rgb;
		cv::Mat _mat_depth8UC1;
		cv::Mat _mat_depth16UC1;

		xn::DepthMetaData _xn_depth_md;
		xn::ImageMetaData _xn_color_md;
		
		XnPoint3D* _point_list;// = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
		XnPoint3D* _real_world;// = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
		pcl::PointCloud<pcl::PointXYZ> _pcl_cloud;


		cv::Mat _main_mask; //-> Arena???

	private:		
		//THREADS
		boost::thread *_t_kinect;
		boost::thread *_t_gui;
		boost::thread *_t_pcl_consumer;
		boost::thread *_t_pcl_producer;

		boost::mutex _mutex_kinect;
		boost::mutex _mutex_pcl;

		boost::mutex _mutex_condition_consumer;
		boost::condition_variable _condition_consumer;

		double  _last_tick[_n_thread_flags];
		int _frame_counter[_n_thread_flags];
		float  _frame_rate[_n_thread_flags];

		
};

#endif//_CONTROLLER