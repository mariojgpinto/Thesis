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

#include "Floor.h"
#include "Mirror.h"
#include "PropertyManager.h"

//#include "..\GUI\SpeculumGUI.h"
#include "..\GUI\GUIController.h"
#include "..\Viewer3D\Viewer3D.h"

#include <NIKinect.h>
#include <ToolBoxPCL.h>

namespace
{
	struct TrackerGroundMouseData
	{
		std::string window_name;
		cv::Mat image;
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
// CONSTRUCTORS
//-----------------------------------------------------------------------------

/**
 * @brief	Controller default constructor.
 * @details	Initializes the variables to NULL, af applicable.
 */
Controller::Controller():
	_mask_main(480,640,CV_8UC1,(const uchar)255),
	_mask_floor(480,640,CV_8UC1,(const uchar)255),
	_mask_mirrors(480,640,CV_8UC1,(const uchar)255),
	_mask_mirrors_and_foor(480,640,CV_8UC1,(const uchar)255),
	_t_kinect(NULL),
	_t_gui(NULL),
	_t_pcl_consumer(NULL),
	_t_pcl_producer(NULL){

	//this->_mutex_kinect = new boost::mutex();
	//this->_mutex_pcl = new boost::mutex();

	for(int i = 0 ; i < this->_n_thread_flags ; ++i){
		_last_tick[i] = 0;
		_frame_counter[i] = 0;
		_frame_rate[i] = 0;
	}

	this->_point_list = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	this->_real_world = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 

	this->_back_3d_to_2d = (int**)malloc(sizeof(int*) * XN_VGA_Y_RES * XN_VGA_X_RES);
	for(int i = 0 ; i < XN_VGA_Y_RES * XN_VGA_X_RES ; i++){
		this->_back_3d_to_2d[i] = (int*)malloc(sizeof(int) * 2);
		this->_back_3d_to_2d[i][XX] = 0;
		this->_back_3d_to_2d[i][YY] = 0;
	}

	this->_back_2d_to_3d = (int**)malloc(sizeof(int*) * XN_VGA_X_RES);
	for(int i = 0 ; i < XN_VGA_X_RES ; ++i){
		 this->_back_2d_to_3d[i] = (int*)malloc(sizeof(int) * XN_VGA_Y_RES);
		 for(int j = 0 ; j < XN_VGA_Y_RES ; ++j){
			 this->_back_2d_to_3d[i][j] = 0;
		 }
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
		this->_kinect->init(oni_file,0,3 + NIKinect::SCENE_A);
	else
		this->_kinect->init();

	//TODO Confirm if everything is ok
	this->_property_manager = new PropertyManager();

	this->_floor = new Floor();

	//this->_mirrors = new std::vector<Mirror*>();
	this->_n_mirrors = 0;

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
	Sleep(1000);
	//this->_t_pcl_producer = new boost::thread(&Controller::thread_pcl_producer,this);
	//Sleep(500);
	this->_t_pcl_consumer = new boost::thread(&Controller::thread_pcl_consumer,this);
	Sleep(500);

	while(this->_property_manager->_running){
		//Copy the information from the Kinect
		this->_mutex_kinect.lock();
			this->process_images();
		this->_mutex_kinect.unlock();

		if(this->_property_manager->_flag_requests[PropertyManager::R_REQUEST]){
			this->process_request();
		}	

		//Creates the prelimenary mask
		this->process_masks();
		
		int pcl_flag = 0;

		if(this->generate_3d()){
			if(this->_property_manager->_flag_processed[PropertyManager::P_MIRROR]){
				this->generate_3d_mirrors();
			}

			if(this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE]){
				this->remove_floor();
			}

			
		}

		if(this->_property_manager->_flag_update[PropertyManager::U_PCL]){
			this->_mutex_pcl.lock();
				this->copy_to_pcl(pcl_flag);
			this->_mutex_pcl.unlock();

			this->_condition_consumer.notify_all();
		}
		if(this->_property_manager->_flag_update[PropertyManager::U_IMAGE]){
			this->_gui->update();
		}

		++this->_frame_counter[Controller::CONTROLLER];
		if (this->_frame_counter[Controller::CONTROLLER] == 15)
		{
			double current_tick = cv::getTickCount();
			this->_frame_rate[Controller::CONTROLLER] = this->_frame_counter[Controller::CONTROLLER] / ((current_tick - this->_last_tick[Controller::CONTROLLER])/cv::getTickFrequency());
			this->_last_tick[Controller::CONTROLLER] = current_tick;
			this->_frame_counter[Controller::CONTROLLER] = 0;
			printf("Frame Rate: %.2f\n",_frame_rate[Controller::CONTROLLER]);
		}
	}

	this->_t_gui->join();
}

void Controller::process_images(){
	this->_kinect->get_color(this->_mat_color_bgr);

	this->_kinect->get_depth(this->_mat_depth16UC1);
	this->_mat_depth16UC1.convertTo(this->_mat_depth8UC1,CV_8UC1);

	this->_kinect->get_depth_meta_data(this->_xn_depth_md);
}

void Controller::process_masks(){
	cv::inRange(this->_mat_depth16UC1,
				this->_property_manager->_depth_min,
				this->_property_manager->_depth_max,
				this->_mask_depth);

	this->_mask_depth.copyTo(this->_mask_main);

	//cv::bitwise_or(this->_mask_depth, this->_mask_mirrors_and_foor,this->_mask_main);
}

void Controller::process_request(){
	if(this->_property_manager->_flag_requests[PropertyManager::R_FLOOR]){
		double a,b,c,d;
		if(this->_kinect->get_floor_plane(&a,&b,&c,&d)){
			printf("%f, %f, %f, %f\n",a,b,c,d);

			this->_floor->set_plane(a,b,c,d);

			this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = true;

			cv::destroyWindow("Add Floor");
		}
		else{
			printf("Floor not found");

			TrackerGroundMouseData ground_mouse_data;
			ground_mouse_data.window_name = "Add Floor";

			this->_mat_color_bgr.copyTo(ground_mouse_data.image);

			cv::imshow(ground_mouse_data.window_name, ground_mouse_data.image);
			cv::setMouseCallback(ground_mouse_data.window_name, on_tracker_ground_mouse, &ground_mouse_data);
			while (cv::waitKey(30) != ' ')
			{
				//QApplication::processEvents();
			}

			cv::destroyWindow(ground_mouse_data.window_name);

			if(ground_mouse_data.points.size() > 2){
				std::vector<cv::Point3f*> points3d;
				XnPoint3D *points_in = (XnPoint3D*)malloc(sizeof(XnPoint3D) * ground_mouse_data.points.size());
				XnPoint3D *points_out = (XnPoint3D*)malloc(sizeof(XnPoint3D) * ground_mouse_data.points.size());

				for(int i = 0 ; i < ground_mouse_data.points.size() ; ++i){
					points_in[i].X = ground_mouse_data.points.at(i)->x;
					points_in[i].Y = ground_mouse_data.points.at(i)->y;
					points_in[i].Z = this->_xn_depth_md[points_in[i].Y * XN_VGA_X_RES + points_in[i].X]; 
				}

				if(! this->_kinect->convert_to_realworld(ground_mouse_data.points.size(),points_in,points_out)){
					this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = false;
					
				}
				else{
					for(int i = 0 ; i < ground_mouse_data.points.size() ; ++i){
						points3d.push_back(new cv::Point3f(points_out[i].X,points_out[i].Y,points_out[i].Z));
					}
				
					ToolBoxPCL::calc_plane_from_points(&points3d,&a,&b,&c,&d,1);

					Floor *floor = new Floor();

					floor->set_plane(a,b,c,d);

					this->add_floor(floor);

					this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = true;
				}
			}
			else{
				this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = false;
			}
		}

		this->_property_manager->_flag_requests[PropertyManager::R_FLOOR] = false;
	}

	if(this->_property_manager->_flag_requests[PropertyManager::R_MIRROR]){
		TrackerGroundMouseData ground_mouse_data;
		ground_mouse_data.window_name = "Add Mirror";

		this->_mat_color_bgr.copyTo(ground_mouse_data.image);

		cv::imshow(ground_mouse_data.window_name, ground_mouse_data.image);
		cv::setMouseCallback(ground_mouse_data.window_name, on_tracker_ground_mouse, &ground_mouse_data);
		while (cv::waitKey(30) != ' ')
		{
			//QApplication::processEvents();
		}

		//cv::destroyWindow(ground_mouse_data.window_name);

		if(ground_mouse_data.points.size() > 2){
			Mirror *mirror = new Mirror();

			mirror->set_area(&ground_mouse_data.points);

			//TODO GET MIRROR PLANE
			TrackerGroundMouseData ground_mouse_data;
			ground_mouse_data.window_name = "Add Mirror";

			this->_mat_color_bgr.copyTo(ground_mouse_data.image,mirror->_area_mask);

			cv::imshow(ground_mouse_data.window_name, ground_mouse_data.image);
			cv::setMouseCallback(ground_mouse_data.window_name, on_tracker_ground_mouse, &ground_mouse_data);
			while (cv::waitKey(30) != ' ')
			{
				//QApplication::processEvents();
			}

			cv::destroyWindow(ground_mouse_data.window_name);

			if(ground_mouse_data.points.size() > 2){
				std::vector<cv::Point3f*> points3d;
				XnPoint3D *points_in = (XnPoint3D*)malloc(sizeof(XnPoint3D) * ground_mouse_data.points.size());
				XnPoint3D *points_out = (XnPoint3D*)malloc(sizeof(XnPoint3D) * ground_mouse_data.points.size());

				for(int i = 0 ; i < ground_mouse_data.points.size() ; ++i){
					if(this->_xn_depth_md[ground_mouse_data.points.at(i)->y * XN_VGA_X_RES + ground_mouse_data.points.at(i)->x] > 1){
						points_in[i].X = ground_mouse_data.points.at(i)->x;
						points_in[i].Y = ground_mouse_data.points.at(i)->y;
						points_in[i].Z = this->_xn_depth_md[points_in[i].Y * XN_VGA_X_RES + points_in[i].X]; 
					}
				}

				if(! this->_kinect->convert_to_realworld(ground_mouse_data.points.size(),points_in,points_out)){
					this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = false;
					
				}
				else{
					for(int i = 0 ; i < ground_mouse_data.points.size() ; ++i){
						points3d.push_back(new cv::Point3f(points_out[i].X,points_out[i].Y,points_out[i].Z));
					}
					double a,b,c,d;
					if(ToolBoxPCL::calc_plane_from_points(&points3d,&a,&b,&c,&d,1)){
						mirror->set_plane(a,b,c,d);

						this->add_mirror(mirror);

						this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = true;
					}
					else{
						this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = false;
					}					
				}
		}
		else{
			this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = false;
		}

		this->_property_manager->_flag_requests[PropertyManager::R_MIRROR] = false;
	}
	}
	if(this->_property_manager->_flag_requests[PropertyManager::R_SAVE]){
		this->save_to_file("..\\..\\data\\config","arena.xml");
		this->_property_manager->_flag_requests[PropertyManager::R_SAVE] = false;
	}

	if(this->_property_manager->_flag_requests[PropertyManager::R_LOAD]){
		this->load_from_file("..\\..\\data\\config\\arena.xml");
		this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = true;
		this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = true;

		this->_property_manager->_flag_requests[PropertyManager::R_LOAD] = false;
	}

	this->_property_manager->_flag_requests[PropertyManager::R_REQUEST] = false;
}


//-----------------------------------------------------------------------------
// GENERATE 3D
//-----------------------------------------------------------------------------

bool Controller::generate_3d(){
	//Create buffer
	this->_n_points = 0;
	uchar* ptr = this->_mask_main.data;
	
	for(int y = 0; y < XN_VGA_Y_RES ; y += this->_property_manager->_3d_step) { 
		for(int x = 0; x < XN_VGA_X_RES ; x += this->_property_manager->_3d_step) { 
			if(ptr[y * XN_VGA_X_RES + x]){
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = this->_xn_depth_md[y * XN_VGA_X_RES + x]; 

				this->_back_3d_to_2d[_n_points][XX] = x;
				this->_back_3d_to_2d[_n_points][YY] = y;
				this->_back_2d_to_3d[x][y] = _n_points;
					
				this->_point_list[_n_points++] = point1;
			}
			
			//else{
			//	this->_back_2d_to_3d[x][y] = -1;
			//}
		}
	} 
	//Convert points	
	return this->_kinect->convert_to_realworld(_n_points, this->_point_list, this->_real_world); 
}

bool Controller::generate_3d_mirrors(){
	uchar* mask_ptr = this->_mask_main.data;

	float dist;
	double nx,ny,nz;
	int idx;

	for(int i = 0 ; i < this->_mirrors.size() ; ++i){
		Mirror *mirror = this->_mirrors[i];

		if(mirror){
			mirror->_n_points = 0;
			mirror->_plane.get_normal(&nx,&ny,&nz);
			nx *= -1; ny *= -1; nz *= -1;

			for(int x = mirror->_area_min_width ; x < mirror->_area_max_width ; ++x){
				for(int y = mirror->_area_min_height ; y < mirror->_area_max_height ; ++y){
					if(mask_ptr[y * XN_VGA_X_RES + x]){
						idx = this->_back_2d_to_3d[x][y];
						XnPoint3D pt = this->_real_world[idx];
						//mirror->_points[mirror->_n_points] = this->_real_world[idx];

						dist = mirror->_plane.distance_to_plane(pt.X,pt.Y,pt.Z);

						pt.X = pt.X + (2 * dist * nx);
						pt.Y = pt.Y + (2 * dist * ny);
						pt.Z = pt.Z + (2 * dist * nz);

						mirror->_points[mirror->_n_points].X = pt.X;
						mirror->_points[mirror->_n_points].Y = pt.Y;
						mirror->_points[mirror->_n_points].Z = pt.Z;
						mirror->_n_points++; 						

						//this->_real_world[idx].X = pt.X;//+= dist * 2 * nx;
						//this->_real_world[idx].Y = pt.Y;//+= dist * 2 * ny;
						//this->_real_world[idx].Z = pt.Z;//+= dist * 2 * nz;
					}
				}
			}
		}
	}

	return true;
}

void Controller::copy_to_pcl(int flag){
	_pcl_cloud.clear();

	switch(flag){
		case 1: //TopView
			{
				//for(int i = 0 ; i < _n_topview_points ; ++i){
				//	pcl::PointXYZRGB pt(255,255,255);
				//	pt.x = this->_real_world[this->_back_topview_to_realworld[i]].X;
				//	pt.y = this->_real_world[this->_back_topview_to_realworld[i]].Y;
				//	pt.z = this->_real_world[this->_back_topview_to_realworld[i]].Z;
				//	this->_pcl_cloud.push_back(pt);
				//}
			}
			break;

		default: //CASE 0
			{
				uchar* ptr = this->_mask_main.data;
				uchar* ptr_floor = this->_mask_floor.data;
				uchar* ptr_mirror = this->_mask_mirrors.data;
				//cv::imshow("win1",this->_mask_mirrors);
				for(int i = 0 ; i < _n_points ; ++i){
					int xx = this->_back_3d_to_2d[i][XX];
					int yy = this->_back_3d_to_2d[i][YY];

					if(ptr[yy * XN_VGA_X_RES + xx]){
						pcl::PointXYZRGB pt(255,255,255);
						//if(ptr_floor[yy * XN_VGA_X_RES + xx]){
						//	pt.r = 0;
						//	pt.b = 0;
						//} else
						if(ptr_mirror[yy * XN_VGA_X_RES + xx]){
							pt.r = 0;
							pt.g = 0;
						}
						
						pt.x = this->_real_world[i].X;
						pt.y = this->_real_world[i].Y;
						pt.z = this->_real_world[i].Z;
						this->_pcl_cloud.push_back(pt);
					}
				}

				for(int i = 0 ; i < this->_mirrors.size() ; ++i){
					Mirror *mirror = this->_mirrors[i];

					if(mirror){
						for(int j = 0 ; j < mirror->_n_points ; ++j){
							pcl::PointXYZRGB pt(255,255,0);
							pt.x = mirror->_points[j].X;
							pt.y = mirror->_points[j].Y;
							pt.z = mirror->_points[j].Z;

							this->_pcl_cloud.push_back(pt);

							pcl::PointXYZRGB pt2(0,255,0);
							pt2.x = mirror->_points_mirrored[j].X;
							pt2.y = mirror->_points_mirrored[j].Y;
							pt2.z = mirror->_points_mirrored[j].Z;

							this->_pcl_cloud.push_back(pt2);
						}
					}
				}
			}
			break;
	}
}

//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
void Controller::remove_floor(){
	uchar* ptr = this->_mask_main.data;
	for(int i = 0 ; i < this->_n_points ; ++i){
		XnPoint3D pt = this->_real_world[i];
		if(pt.Z > 0.0){
			int xx = this->_back_3d_to_2d[i][XX];
			int yy = this->_back_3d_to_2d[i][YY];

			double dist = this->_floor->_plane.distance_to_plane(pt.X,pt.Y,pt.Z);
			if(dist < this->_floor->_thresh){
				ptr[yy * XN_VGA_X_RES + xx] = 0;
			}
		}
	}
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
						//this->_pcl_cloud.push_back(pcl::PointXYZ(	this->_real_world[i].X,
						//											this->_real_world[i].Y,
						//											this->_real_world[i].Z));
						pcl::PointXYZRGB pt(255,255,255);
						pt.x = this->_real_world[i].X;
						pt.y = this->_real_world[i].Y;
						pt.z = this->_real_world[i].Z;
					
						//cloud.points[y * XN_VGA_X_RES + x] = pt;
						this->_pcl_cloud.push_back(pt);
					
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
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

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