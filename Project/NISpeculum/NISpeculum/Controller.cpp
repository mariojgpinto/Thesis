/** 
 * @file	Controller.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	July, 2013 
 * @brief	Implementation of the Controller Class.
 *
 * @details	Detailed Information.
 */
#include "Controller.h"

#undef max
#undef min

#include <NIKinect.h>
#include "PropertyManager.h"
//#include "..\GUI\SpeculumGUI.h"
#include "..\GUI\GUIController.h"
#include "..\Viewer3D\Viewer3D.h"


//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------

/**
 * @brief	Controller default constructor.
 * @details	Initializes the variables to NULL, af applicable.
 */
Controller::Controller(){
	this->_t_kinect = NULL;
	this->_t_gui = NULL;
	this->_t_pcl_consumer = NULL;
	this->_t_pcl_producer = NULL;

	//this->_mutex_kinect = new boost::mutex();
	//this->_mutex_pcl = new boost::mutex();

	for(int i = 0 ; i < this->_n_thread_flags ; ++i){
		_last_tick[i] = 0;
		_frame_counter[i] = 0;
		_frame_rate[i] = 0;
	}
}

/**
 * @brief	Controller destructor.
 * @details	
 */
Controller::~Controller(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void Controller::init(const char* oni_file){
	this->_kinect = new NIKinect();

	if(oni_file)
		this->_kinect->init(oni_file);
	else
		this->_kinect->init();

	//TODO Confirm if everything is ok

	this->_point_list = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	this->_real_world = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 

	this->_property_manager = new PropertyManager();

	this->_3d_viewer = new Viewer3D();

	this->_gui = new GUIController(this);

}

//-----------------------------------------------------------------------------
// RUNNING
//-----------------------------------------------------------------------------
void Controller::run(int argc, char* argv[]){
	this->_t_gui = new boost::thread(&Controller::thread_gui,this,argc,argv);
	Sleep(500);

	this->_t_kinect = new boost::thread(&Controller::thread_kinect,this);
	Sleep(500);
	this->_t_pcl_producer = new boost::thread(&Controller::thread_pcl_producer,this);
	Sleep(500);
	this->_t_pcl_consumer = new boost::thread(&Controller::thread_pcl_consumer,this);
	Sleep(500);
	
	//this->thread_gui(argc,argv);
	
	
	while(this->_property_manager->_running){
		this->_mutex_kinect.lock();
			this->_kinect->get_color(this->_mat_color_bgr);
			this->_kinect->get_depth(this->_mat_depth16UC1);	
		this->_mutex_kinect.unlock();

		this->_mat_depth16UC1.convertTo(this->_mat_depth8UC1,CV_8UC1);

		this->_gui->update(&this->_mat_color_bgr);

		cv::waitKey(50);
	}

	this->_t_gui->join();
}

//-----------------------------------------------------------------------------
// THREAD
//-----------------------------------------------------------------------------
/**
 * @brief	Function for the Kinect Thread execution.
 * @details	
 */
void Controller::thread_kinect(){
	while(_property_manager->_running){
		this->_kinect->update_openni();

		this->_mutex_kinect.lock();
			this->_kinect->update_images();
		this->_mutex_kinect.unlock();

		++this->_frame_counter[Controller::KINECT];
		if (this->_frame_counter[Controller::KINECT] == 15)
		{
			double current_tick = cv::getTickCount();
			this->_frame_rate[Controller::KINECT] = this->_frame_counter[Controller::KINECT] / ((current_tick - this->_last_tick[Controller::KINECT])/cv::getTickFrequency());
			this->_last_tick[Controller::KINECT] = current_tick;
			this->_frame_counter[Controller::KINECT] = 0;
			//printf("\t\t\t PCLProducer FrameRate %.2f\n",_frame_rate);
		}
	}
}

/**
 * @brief	Function for the Kinect Thread execution.
 * @details	
 */
void Controller::thread_pcl_producer(){
	while(this->_property_manager->_running){
		//Get copy of MetaData
		this->_mutex_kinect.lock();
			//copy MD
			this->_kinect->get_depth_meta_data(this->_xn_depth_md);
		this->_mutex_kinect.unlock();

		//Create buffer
		int n_points = 0;
		for(int y = 0; y < XN_VGA_Y_RES ; y += this->_property_manager->_3d_step) { 
			for(int x = 0; x < XN_VGA_X_RES ; x += this->_property_manager->_3d_step) { 
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = this->_xn_depth_md[y * XN_VGA_X_RES + x]; 

				this->_point_list[n_points++] = point1;
				//this->_point_list[y * XN_VGA_X_RES + x] = point1;
			}
		} 
		//Convert points	
		bool result = this->_kinect->convert_to_realworld(n_points, this->_point_list, this->_real_world); 

		if(result){
			this->_mutex_pcl.lock();

				_pcl_cloud.clear();
				for(int i = 0 ; i < n_points ; ++i){
					if(this->_real_world[i].Z > 0.0){
						this->_pcl_cloud.push_back(pcl::PointXYZ(	this->_real_world[i].X,
																	this->_real_world[i].Y,
																	this->_real_world[i].Z));
					//pcl::PointXYZRGB pt(ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 2],
					//					ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 1],
					//					ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 0]);
					//pt.x = _real_world[y * XN_VGA_X_RES + x].X;
					//pt.y = _real_world[y * XN_VGA_X_RES + x].Y;
					//pt.z = _real_world[y * XN_VGA_X_RES + x].Z;
					//
					////cloud.points[y * XN_VGA_X_RES + x] = pt;
					//cloud.push_back(pt);
					
			//		ac++;
					}
				}

			this->_mutex_pcl.unlock();

			this->_condition_consumer.notify_all();
		}
		
		++_frame_counter[Controller::PCL_PRODUCER];
		if (_frame_counter[Controller::PCL_PRODUCER] == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate[Controller::PCL_PRODUCER] = _frame_counter[Controller::PCL_PRODUCER] / ((current_tick - _last_tick[Controller::PCL_PRODUCER])/cv::getTickFrequency());
			_last_tick[Controller::PCL_PRODUCER] = current_tick;
			_frame_counter[Controller::PCL_PRODUCER] = 0;
			printf("\t\t\t PCLProducer FrameRate %.2f\n",_frame_rate[Controller::PCL_PRODUCER]);
		}
	}
}

/**
 * @brief	Function for the Kinect Thread execution.
 * @details	
 */
void Controller::thread_pcl_consumer(){
	pcl::PointCloud<pcl::PointXYZ> cloud;

	while(_property_manager->_running){
		//Wait for PointCloud
		{
			boost::mutex::scoped_lock lock(this->_mutex_condition_consumer);
			this->_condition_consumer.wait(lock);
		}

		this->_mutex_pcl.lock();
			cloud = this->_pcl_cloud;
		this->_mutex_pcl.unlock();

		this->_3d_viewer->show_cloud(&cloud);

		++_frame_counter[Controller::PCL_CONSUMER];
		if (_frame_counter[Controller::PCL_CONSUMER] == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate[Controller::PCL_CONSUMER] = _frame_counter[Controller::PCL_CONSUMER] / ((current_tick - _last_tick[Controller::PCL_CONSUMER])/cv::getTickFrequency());
			_last_tick[Controller::PCL_CONSUMER] = current_tick;
			_frame_counter[Controller::PCL_CONSUMER] = 0;
			//printf("\t\t\t PCLProducer FrameRate %.2f\n",_frame_rate);
		}
	}
}

void Controller::thread_gui(int argc, char* argv[]){
	this->_gui->run(argc, argv);
}