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


//OUTLIERS
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//SMOOTHING
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

//TRIANGULATION
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//VOXEL
#include <pcl/filters/voxel_grid.h>

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------

/**
 * @brief	Controller default constructor.
 * @details	Initializes the variables to NULL, af applicable.
 */
Controller::Controller():
	_mask_depth(480,640,CV_8UC1,(const uchar)255),
	_mask_main(480,640,CV_8UC1,(const uchar)255),
	_mask_floor(480,640,CV_8UC1,(const uchar)255),
	_mask_mirrors(480,640,CV_8UC1,(const uchar)255),
	_mask_mirrors_and_foor(480,640,CV_8UC1,(const uchar)255),
	_t_kinect(NULL),
	_t_gui(NULL),
	_t_pcl_consumer(NULL),
	_t_pcl_producer(NULL),
	_t_pcl_polygon(NULL){

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

	this->_aux_points = new std::vector<cv::Point*>();
	
	this->_model_n_frames = 0;
	this->_model_n_points = 0;
	this->_model_projective = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES);
	this->_model_realworld = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES);

	this->_model_back_3d_to_2d = (int**)malloc(sizeof(int*) * XN_VGA_Y_RES * XN_VGA_X_RES);
	for(int i = 0 ; i < XN_VGA_Y_RES * XN_VGA_X_RES ; i++){
		this->_model_back_3d_to_2d[i] = (int*)malloc(sizeof(int) * 2);
		this->_model_back_3d_to_2d[i][XX] = 0;
		this->_model_back_3d_to_2d[i][YY] = 0;
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
	Sleep(1500);

	this->_t_pcl_polygon = new boost::thread(&Controller::thread_pcl_polygon,this);
	Sleep(500);

	while(this->_property_manager->_running){
		//Copy the information from the Kinect
		if(!this->_property_manager->_pause){
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
					this->process_mirrors_masks();
					this->generate_3d_mirrors();
					pcl_flag += PropertyManager::P_MIRROR;
				}

				if(this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE]){
					this->process_floor_mask();
					this->remove_floor();
					pcl_flag += PropertyManager::P_FLOOR_PLANE;
				}
			
				this->_mutex_pcl.lock();
					this->copy_to_pcl(pcl_flag);
				this->_mutex_pcl.unlock();

				if(this->_property_manager->_flag_processed[PropertyManager::P_VOXEL]){
					//Add Voxel Grid Filter
					// Create the filtering object
					sensor_msgs::PointCloud2 cloud2; 
					pcl::toROSMsg(this->_pcl_cloud,cloud2);
					sensor_msgs::PointCloud2ConstPtr cloud2Ptr (new sensor_msgs::PointCloud2(cloud2)); 
					//const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud(((const pcl::PointCloud<pcl::PointXYZRGB>)this->_pcl_cloud).makeShared());
					//pcl::PointCloudPtr downsampled;
					//pcl::PointCloud<pcl::PointXYZ> cloud;// = this->_pcl_cloud;

					pcl::VoxelGrid<sensor_msgs::PointCloud2> sor; 
					sor.setInputCloud (cloud2Ptr);
					sor.setLeafSize (	this->_property_manager->_voxel_grid_x, 
										this->_property_manager->_voxel_grid_y, 
										this->_property_manager->_voxel_grid_z);

					sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ()); 
					sor.filter (*cloud_filtered); 
        
					pcl::fromROSMsg (*cloud_filtered, this->_pcl_cloud); 
				}

				if(this->_property_manager->_flag_processed[PropertyManager::P_OUTLIERS]){
					if(this->_property_manager->_outliers_methods == 1){
					
						pcl::PointCloud<pcl::PointXYZRGB> cloud = this->_pcl_cloud;
						pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
						// build the filter
						outrem.setInputCloud(cloud.makeShared());
						outrem.setRadiusSearch(this->_property_manager->_outliers_radius_radius);
						outrem.setMinNeighborsInRadius (this->_property_manager->_outliers_radius_neighbors);
						// apply filter
						outrem.setNegative(true);
						outrem.filter (this->_pcl_cloud);
					} 
						else
					if(this->_property_manager->_outliers_methods == 2){
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = this->_pcl_cloud.makeShared();
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

						// Create the filtering object
						pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
						sor.setInputCloud (cloud);
						sor.setMeanK (this->_property_manager->_outliers_statistical_meank);
						sor.setStddevMulThresh (this->_property_manager->_outliers_statistical_stddev);
						sor.filter (*cloud_filtered);

						//sor.setNegative (true);
						//sor.filter (*cloud_filtered);
						printf("Statistical\n");

						this->_pcl_cloud = *cloud_filtered.get();
					}
					
				}

				if(this->_property_manager->_flag_processed[PropertyManager::P_CAPTURE]){
					if(this->_model_frames_depth.size() < this->_model_n_frames){
						this->model_add_frame();
					}
					else{
						this->generate_model();
						this->_property_manager->_flag_processed[PropertyManager::P_CAPTURE] = false;
					}
				}

				if(this->_property_manager->_flag_processed[PropertyManager::P_POLYGON]){
					if(this->_polygon_ready_new){
						pcl::copyPointCloud(this->_pcl_cloud,this->_pcl_cloud_polygon);

						this->_polygon_ready_new = false;

						this->_condition_polygon.notify_all();
					}
					//this->generate_polygon();
				}
			}

			if(this->_property_manager->_flag_update[PropertyManager::U_PCL]){
				

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
		else{
			this->_condition_consumer.notify_all();
		}
	}

	this->_t_gui->join();
}

void Controller::process_images(){
	this->_kinect->get_color(this->_mat_color_bgr);

	this->_kinect->get_depth(this->_mat_depth16UC1);
	this->_mat_depth16UC1.convertTo(this->_mat_depth8UC1,CV_8UC1,0.05);

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

void Controller::process_floor_mask(){
	cv::Mat temp;
	this->_mat_depth16UC1.copyTo(temp,this->_floor->_area_mask);
	cv::inRange(temp,
				this->_floor->_depth_min,
				this->_floor->_depth_max,
				this->_floor->_mask);
}

void Controller::process_mirrors_masks(){
	for(int i = 0 ; i < this->_mirrors.size() ; ++i){
		Mirror* mirror = this->_mirrors[i];

		cv::Mat temp;
		this->_mat_depth16UC1.copyTo(temp,mirror->_area_mask);
		cv::inRange(temp,
					mirror->_depth_min,
					mirror->_depth_max,
					mirror->_mask);

		//cv::imshow("win1",mirror->_mask);
		//cv::waitKey();
	}
}

void Controller::process_request(){
	if(this->_property_manager->_flag_requests[PropertyManager::R_FLOOR_AREA]){
		if(this->_aux_points->size() > 2){
			Floor *floor = new Floor();
			floor->set_area(this->_aux_points);

			this->add_floor(floor);

			this->_gui->update_floor_manager();
		}
		this->_property_manager->_flag_requests[PropertyManager::R_FLOOR_AREA] = false;
	}
	
	if(this->_property_manager->_flag_requests[PropertyManager::R_FLOOR_POINTS]){
		double a,b,c,d;
		if(this->_kinect->get_floor_plane(&a,&b,&c,&d)){
			printf("%f, %f, %f, %f\n",a,b,c,d);

			this->_floor->set_plane(a,b,c,d);

			this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = true;

			cv::destroyWindow("Add Floor");
		}
		else{
			if(this->_aux_points->size() > 2){
				std::vector<cv::Point3f*> points3d;
				XnPoint3D *points_in = (XnPoint3D*)malloc(sizeof(XnPoint3D) * _aux_points->size());
				XnPoint3D *points_out = (XnPoint3D*)malloc(sizeof(XnPoint3D) * _aux_points->size());

				for(int i = 0 ; i < _aux_points->size() ; ++i){
					points_in[i].X = _aux_points->at(i)->x;
					points_in[i].Y = _aux_points->at(i)->y;
					points_in[i].Z = this->_xn_depth_md[points_in[i].Y * XN_VGA_X_RES + points_in[i].X]; 
				}

				if(! this->_kinect->convert_to_realworld(_aux_points->size(),points_in,points_out)){
					this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = false;
					
				}
				else{
					for(int i = 0 ; i < _aux_points->size() ; ++i){
						points3d.push_back(new cv::Point3f(points_out[i].X,points_out[i].Y,points_out[i].Z));
					}
				
					ToolBoxPCL::calc_plane_from_points(&points3d,&a,&b,&c,&d,1);

					this->_floor->set_plane(a,b,c,d);
					
					this->_gui->update_floor_manager();

					this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = true;
				}
			}
			else{
				this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = false;
			}
		}

		this->_property_manager->_flag_requests[PropertyManager::R_FLOOR_POINTS] = false;
	}

	if(this->_property_manager->_flag_requests[PropertyManager::R_MIRROR_AREA]){
		if(this->_aux_points->size() > 2){
			Mirror *mirror = new Mirror();

			mirror->set_area(this->_aux_points);

			this->add_mirror(mirror);

			this->_gui->update_mirror_manager();
			//this->_gui->point_selection(3);
		}
		this->_property_manager->_flag_requests[PropertyManager::R_MIRROR_AREA] = false;
	}
	
	if(this->_property_manager->_flag_requests[PropertyManager::R_MIRROR_POINTS]){
		if(this->_aux_points->size() > 2){
			Mirror *mirror = this->_mirrors[this->_mirrors.size()-1];

			std::vector<cv::Point3f*> points3d;
			XnPoint3D *points_in = (XnPoint3D*)malloc(sizeof(XnPoint3D) * this->_aux_points->size());
			XnPoint3D *points_out = (XnPoint3D*)malloc(sizeof(XnPoint3D) * this->_aux_points->size());

			for(int i = 0 ; i < this->_aux_points->size() ; ++i){
				if(this->_xn_depth_md[this->_aux_points->at(i)->y * XN_VGA_X_RES + this->_aux_points->at(i)->x] > 1){
					points_in[i].X = this->_aux_points->at(i)->x;
					points_in[i].Y = this->_aux_points->at(i)->y;
					points_in[i].Z = this->_xn_depth_md[points_in[i].Y * XN_VGA_X_RES + points_in[i].X]; 
				}
			}

			if(! this->_kinect->convert_to_realworld(this->_aux_points->size(),points_in,points_out)){
				this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = false;
					
			}
			else{
				for(int i = 0 ; i < this->_aux_points->size() ; ++i){
					points3d.push_back(new cv::Point3f(points_out[i].X,points_out[i].Y,points_out[i].Z));
				}
				double a,b,c,d;
				if(ToolBoxPCL::calc_plane_from_points(&points3d,&a,&b,&c,&d,1)){
					mirror->set_plane(a,b,c,d);

					//this->add_mirror(mirror);

					this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = true;

					this->_gui->update_mirror_manager();
				}
				else{
					this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = false;
				}					
			}
		}
		else{
			this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = false;
		}
	}

	if(this->_property_manager->_flag_requests[PropertyManager::R_SAVE]){
		this->save_to_file("..\\..\\data\\config","arena");
		this->_property_manager->_flag_requests[PropertyManager::R_SAVE] = false;
	}

	if(this->_property_manager->_flag_requests[PropertyManager::R_LOAD]){
		this->load_from_file("..\\..\\data\\config\\arena.xml");
		this->_gui->update_mirror_manager();
		this->_property_manager->_flag_processed[PropertyManager::P_MIRROR] = true;
		this->_property_manager->_flag_processed[PropertyManager::P_FLOOR_PLANE] = true;

		this->_property_manager->_flag_requests[PropertyManager::R_LOAD] = false;
	}

	if(this->_property_manager->_flag_requests[PropertyManager::R_CAPTURE]){
		this->_model_frames_depth.clear();
		this->_model_frames_color.clear();
		this->_model_cloud.clear();
		this->_model_n_points = 0;

		this->_property_manager->_flag_processed[PropertyManager::P_CAPTURE] = true;

		this->_property_manager->_flag_requests[PropertyManager::R_CAPTURE] = false;
	}

	if(this->_property_manager->_flag_requests[PropertyManager::R_SAVE_PCL]){
		if(this->_property_manager->_flag_update[PropertyManager::U_PCL]){
			pcl::PointCloud<pcl::PointXYZRGB> cloud;

			if(this->_property_manager->_flag_processed[PropertyManager::P_CAPTURE_SHOW]){
				//Model Point Cloud
				cloud = this->_model_cloud;
			}
			else{
				//Current Point Cloud
				cloud = this->_pcl_cloud;
			}

			if(this->_property_manager->_save_pcl_mode == 1){
				ToolBoxPCL::write_to_ply_file(cloud,this->_property_manager->_file_name);
			} else
			if(this->_property_manager->_save_pcl_mode == 1){
				ToolBoxPCL::write_to_pcd_file(cloud,this->_property_manager->_file_name);
			}else
			if(this->_property_manager->_save_pcl_mode == 1){
				ToolBoxPCL::write_to_obj_file(cloud,this->_property_manager->_file_name);
			}
			
			this->_property_manager->_flag_requests[PropertyManager::R_SAVE_PCL] = false;


		}
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

	//Create buffer
	//Add Filter on 3D Manager Window! 
	if(this->_property_manager->_flag_processed[PropertyManager::P_FILTER]){
		//Bilateral
		if(this->_property_manager->_filter_method == 1){
			cv::Mat temp;cv::Mat temp2 = cv::Mat(480,640,CV_16UC1);;
			this->_mat_depth16UC1.convertTo(temp,CV_32FC1);
			cv::bilateralFilter(temp,temp2,5,4,4);

			temp2.convertTo(this->_mat_depth16UC1,CV_16UC1);
		} else
		//Median
		if(this->_property_manager->_filter_method == 2){
			cv::medianBlur(this->_mat_depth16UC1,this->_mat_depth16UC1,5);
		} else
		//Gaussian
		if(this->_property_manager->_filter_method == 3){
			cv::GaussianBlur(this->_mat_depth16UC1,this->_mat_depth16UC1,cv::Size(5,5),0);
		}
	}
	//In the pass through filtering, all points with matching values in a predefined value range are passed through 
	//without any modifications to their data. The pass through filtering is one of the basic filtering methods and 
	//it is fast and simple to use. It is implemented in PCL as a template class and therefore supports all predefined 
	//point types. (29.)
	//CV::INPAINT -> Active Segmentation in 3D using iKnect Sensor

	//cv::GaussianBlur(this->_mat_depth16UC1,this->_mat_depth16UC1,cv::Size(3,3),0);
	UINT16* ptr_16u = (UINT16*)this->_mat_depth16UC1.data;


	for(int y = 0; y < XN_VGA_Y_RES ; y += this->_property_manager->_3d_step) { 
		for(int x = 0; x < XN_VGA_X_RES ; x += this->_property_manager->_3d_step) { 
			if(ptr[y * XN_VGA_X_RES + x]){
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = ptr_16u[y * XN_VGA_X_RES + x]; 

				this->_back_3d_to_2d[_n_points][XX] = x;
				this->_back_3d_to_2d[_n_points][YY] = y;
				this->_back_2d_to_3d[x][y] = _n_points;
					
				this->_point_list[_n_points++] = point1;
			}
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
	//uint8_t* ptr_clr = (uint8_t*)this->_mat_color_bgr.data;
	//int xx, yy;
	for(int i = 0 ; i < this->_mirrors.size() ; ++i){
		Mirror *mirror = this->_mirrors[i];

		if(mirror){
			mirror->_n_points = 0;
			mirror->_plane.get_normal(&nx,&ny,&nz);
			nx *= -1; ny *= -1; nz *= -1;

			uchar* mask_mirror_ptr = mirror->_mask.data;
			//cv::imshow("win2",mirror->_mask);

			for(int x = mirror->_area_min_width ; x < mirror->_area_max_width ; x+=this->_property_manager->_3d_step){
				for(int y = mirror->_area_min_height ; y < mirror->_area_max_height ; y+=this->_property_manager->_3d_step){
					idx = y * XN_VGA_X_RES + x;
					if(mask_ptr[idx] && mask_mirror_ptr[idx]){
						idx = this->_back_2d_to_3d[x][y];
						XnPoint3D pt = this->_real_world[idx];
						//mirror->_points[mirror->_n_points] = this->_real_world[idx];

						//xx = this->_back_3d_to_2d[idx][XX];
						//yy = this->_back_3d_to_2d[idx][YY];

						mirror->_points[mirror->_n_points].X = pt.X;
						mirror->_points[mirror->_n_points].Y = pt.Y;
						mirror->_points[mirror->_n_points].Z = pt.Z;

						dist = mirror->_plane.distance_to_plane(pt.X,pt.Y,pt.Z);

						pt.X = pt.X + (2 * dist * nx);
						pt.Y = pt.Y + (2 * dist * ny);
						pt.Z = pt.Z + (2 * dist * nz);

						double dist = this->_floor->_plane.distance_to_plane(pt.X,pt.Y,pt.Z);
						if(dist > this->_floor->_thresh){
							mirror->_points_mirrored[mirror->_n_points].X = pt.X;
							mirror->_points_mirrored[mirror->_n_points].Y = pt.Y;
							mirror->_points_mirrored[mirror->_n_points].Z = pt.Z;

							mirror->_points_idx[mirror->_n_points] = idx;
							mirror->_n_points++; 

							//ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 2] = 255;
							//ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 1] = 0;
							//ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 0] = 0;
						}
					}
				}
			}
		}
	}

	return true;
}

void Controller::generate_polygon(){
	//MP SEARCH FOR FAST MESH GENERATION AND PCL VOXEL GRID
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(this->_pcl_cloud_polygon, *vertices);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
	ne.setRadiusSearch (this->_property_manager->_3d_normal_radius);
	ne.setInputCloud (this->_pcl_cloud_polygon.makeShared());
	ne.compute (*vertices);

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>); 
	tree2->setInputCloud (vertices); 

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;


	/*
	setMaximumNearestNeighbors(unsigned) and setMu(double) control the size of the neighborhood. 
		The former defines how many neighbors are searched for, while the latter specifies the maximum acceptable distance 
		for a point to be considered, relative to the distance of the nearest point (in order to adjust to changing densities). 
		Typical values are 50-100 and 2.5-3 (or 1.5 for grids).
	setSearchRadius(double) is practically the maximum edge length for every triangle. 
		This has to be set by the user such that to allow for the biggest triangles that should be possible.
	setMinimumAngle(double) and setMaximumAngle(double) are the minimum and maximum angles in each triangle. 
		While the first is not guaranteed, the second is. Typical values are 10 and 120 degrees (in radians).
	setMaximumSurfaceAgle(double) and setNormalConsistency(bool) are meant to deal with the cases where there are sharp 
		edges or corners and where two sides of a surface run very close to each other. To achieve this, points are not 
		connected to the current point if their normals deviate more than the specified angle 
		(note that most surface normal estimation methods produce smooth transitions between normal angles even at sharp edges). 
		This angle is computed as the angle between the lines defined by the normals (disregarding the normal’s direction) 
		if the normal-consistency-flag is not set, as not all normal estimation methods can guarantee consistently oriented normals. 
		Typically, 45 degrees (in radians) and false works on most datasets.
	*/


	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (10);

	// Set typical values for the parameters
	gp3.setMu (15);
	gp3.setMaximumNearestNeighbors (200);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(true);

	// Get result
	gp3.setInputCloud (vertices);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	this->_polygon = triangles;
}

void Controller::model_add_frame(){
	cv::Mat temp_depth;
	cv::Mat temp_color;

	this->_mat_depth16UC1.copyTo(temp_depth);
	this->_mat_color_bgr.copyTo(temp_color);

	this->_model_frames_depth.push_back(temp_depth);
	this->_model_frames_color.push_back(temp_color);
}

cv::RNG rng(12345);

void Controller::generate_model(){
	//Create Average images;
	//this->_model_depth_average = cv::Mat::zeros(cv::Size(640,480),CV_16UC1);
	//this->_model_color_average = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
	//cv::Mat average_counter = cv::Mat::zeros(cv::Size(640,480),CV_8UC1);
	int old_step = this->_property_manager->_3d_step;

	cv::Mat1i freq_n(cv::Size(640,480),0);
	cv::Mat1f freq_v(cv::Size(640,480),0);
	cv::Mat freq_c = cv::Mat::zeros(cv::Size(640,480),CV_32FC3);

	int idx, idx_clr;
	float* ptr_freq_v = (float*)freq_v.data;
	int* ptr_freq_n = (int*)freq_n.data;
	float* ptr_freq_c = (float*)freq_c.data;

	//Create Depth and Color Sum Image
	for(int k = 0 ; k < this->_model_frames_depth.size() ; ++k){
		UINT16* ptr_16u = (UINT16*)this->_model_frames_depth[k].data;
		uint8_t* ptr_clr = (uint8_t*)this->_model_frames_color[k].data;
		for(int y = 0; y < XN_VGA_Y_RES ; ++y) { 
			for(int x = 0; x < XN_VGA_X_RES ; ++x) { 
				idx = y * XN_VGA_X_RES + x;
				if(ptr_16u[idx]){
					ptr_freq_n[idx]++;
					ptr_freq_v[idx]+=ptr_16u[idx];
				}

				idx_clr = y * XN_VGA_X_RES * 3 + x* 3;

				ptr_freq_c[idx_clr + 0] += ptr_clr[idx_clr + 0];
				ptr_freq_c[idx_clr + 1] += ptr_clr[idx_clr + 1];
				ptr_freq_c[idx_clr + 2] += ptr_clr[idx_clr + 2];
			} 
		}
	}

	//Create Depth and Color Average Image
	for(int y = 0; y < XN_VGA_Y_RES ; ++y) { 
		for(int x = 0; x < XN_VGA_X_RES ; ++x) {
			idx = y * XN_VGA_X_RES + x;
			if(ptr_freq_n[idx]){
				ptr_freq_v[idx] /= ptr_freq_n[idx];
			}
		}
	}

	freq_c /= this->_model_frames_color.size();
	
	
	freq_c.convertTo(this->_model_color_average,CV_8UC3);
	freq_v.convertTo(this->_model_depth_average,CV_16UC1);


	//cv::Mat t8u;
	//this->_model_depth_average.convertTo(t8u,CV_8UC1,0.05);
	//cv::imshow("win1",this->_model_color_average);
	////cv::imshow("win2",this->_mat_color_bgr);
	//cv::imshow("win3",t8u);
	//cv::imshow("win4",this->_mat_depth8UC1);
	//cv::waitKey();
	
	//Create Masks for avg images
	cv::Mat main_mask;

	cv::inRange(this->_model_depth_average,
				this->_property_manager->_depth_min,
				this->_property_manager->_depth_max,
				main_mask);

	cv::bitwise_and(main_mask,this->_mask_mirrors_and_foor,main_mask);


	//Contours to remove noise
	cv::Mat t8u;
	main_mask.convertTo(t8u,CV_8UC1,0.05);
	//cv::Mat image_temp;
	//this->_model_depth_average.copyTo(image_temp);

	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
			
	cv::findContours( t8u, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	for(unsigned int i = 0; i< contours.size(); i++ ){
		if(contours[i].size() < 50){
			cv::Scalar clr = cv::Scalar(/*255);// */rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			cv::Rect rect = cv::boundingRect(contours[i]);
			drawContours( this->_model_color_average, contours, i, clr, -1, 8, hierarchy, 0, cv::Point() );
			//cv::rectangle(top_view_color,rect, clr);
		}
	}

	Mirror* mirror;
	uchar* ptr_mirror_mask = this->_mask_mirrors.data;

	//Generate 3D
	this->_model_n_points;

	uchar* ptr_main_mask = main_mask.data;
	//uchar* ptr_mirror_mask = this->_mask_mirrors.data;

	UINT16* ptr_16u = (UINT16*)this->_model_depth_average.data;

	for(int y = 0; y < XN_VGA_Y_RES ; ++y /*y += this->_property_manager->_3d_step*/) { 
		for(int x = 0; x < XN_VGA_X_RES ; ++x/*x += this->_property_manager->_3d_step*/) { 
			idx = y * XN_VGA_X_RES + x;

			if(ptr_main_mask[idx]){
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = ptr_16u[idx];

				if(ptr_mirror_mask[idx]){
					for(int j = 0 ; j < this->_mirrors.size() ; ++j){
						mirror = this->_mirrors[j];

						if(mirror && mirror->_area_mask.data[idx]){
							if(point1.Z > mirror->_depth_min && point1.Z < mirror->_depth_max){
								this->_model_back_3d_to_2d[this->_model_n_points][XX] = x;
								this->_model_back_3d_to_2d[this->_model_n_points][YY] = y;
				
								this->_model_projective[this->_model_n_points++] = point1;
							}
						}
					}
				}
				else{
					this->_model_back_3d_to_2d[this->_model_n_points][XX] = x;
					this->_model_back_3d_to_2d[this->_model_n_points][YY] = y;
				
					this->_model_projective[this->_model_n_points++] = point1;
				}
			}
		}
	} 

	bool result = this->_kinect->convert_to_realworld(	this->_model_n_points, 
														this->_model_projective,
														this->_model_realworld); 

	
	//uint8_t* ptr_clr = (uint8_t*)this->_model_color_average.data;

	//Generate PCL-3D Floor
	//uchar* ptr_mask_floor = mask_floor.data;
	//for(int y = 0; y < XN_VGA_Y_RES ; y += this->_property_manager->_3d_step) { 
	//	for(int x = 0; x < XN_VGA_X_RES ; x += this->_property_manager->_3d_step) { 
	//		//if(ptr_mask_floor[y * XN_VGA_X_RES + x]){

	//		//}
	//	}
	//}

	//Generate PCL-3D from Mirrors
	_pcl_cloud.clear();

	uchar* ptr = this->_mask_main.data;

	uint8_t* ptr_clr = (uint8_t*)this->_model_color_average.data;

	_model_cloud.clear();

	int xx, yy;
	double dist;
	
	std::vector<double> nx,ny,nz;
	for(int i = 0 ; i < this->_mirrors.size() ; ++i){
		double nx_,ny_,nz_;

		mirror = this->_mirrors[i];

		if(mirror){
			mirror->_n_points = 0;
			mirror->_plane.get_normal(&nx_,&ny_,&nz_);
			nx_ *= -1; ny_ *= -1; nz_ *= -1;

			nx.push_back(nx_);
			ny.push_back(ny_);
			nz.push_back(nz_);
		}
	}


	for(int i = 0 ; i < this->_model_n_points ; ++i){
		xx = this->_model_back_3d_to_2d[i][XX];
		yy = this->_model_back_3d_to_2d[i][YY];
		idx = yy * XN_VGA_X_RES + xx;

		if(ptr_mirror_mask[idx]){
			for(int j = 0 ; j < this->_mirrors.size() ; ++j){
				mirror = this->_mirrors[j];

				if(mirror && mirror->_area_mask.data[idx]){
					//if(this->_model_realworld[i].Z > mirror->_depth_min && this->_model_realworld[i].Z < mirror->_depth_max){
						dist = mirror->_plane.distance_to_plane( this->_model_realworld[i].X,
																 this->_model_realworld[i].Y,
																 this->_model_realworld[i].Z);

						this->_model_realworld[i].X += 2 * dist * nx[j];
						this->_model_realworld[i].Y += 2 * dist * ny[j];
						this->_model_realworld[i].Z += 2 * dist * nz[j];
					//}
				}
			}
		}


		dist = this->_floor->_plane.distance_to_plane(	this->_model_realworld[i].X,
														this->_model_realworld[i].Y,
														this->_model_realworld[i].Z);

		if(dist > this->_floor->_thresh){
			

			pcl::PointXYZRGB pt(ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 2],
								ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 1],
								ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 0]);
						
			pt.x = this->_model_realworld[i].X;
			pt.y = this->_model_realworld[i].Y;
			pt.z = this->_model_realworld[i].Z;
			this->_model_cloud.push_back(pt);
		}
	}

	this->_property_manager->_3d_step = old_step;

	this->_property_manager->_flag_processed[PropertyManager::P_CAPTURE_SHOW] = true;
}

void Controller::copy_to_pcl(int flag){
	_pcl_cloud.clear();

	uchar* ptr = this->_mask_main.data;
	uint8_t* ptr_clr = (uint8_t*)this->_mat_color_bgr.data;

	switch(flag){
		case 0: //FULL IMAGE
			{
				for(int i = 0 ; i < _n_points ; ++i){
					int xx = this->_back_3d_to_2d[i][XX];
					int yy = this->_back_3d_to_2d[i][YY];

					if(ptr[yy * XN_VGA_X_RES + xx]){
						
						pcl::PointXYZRGB pt(ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 2],
												 ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 1],
												 ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 0]);
						
						pt.x = this->_real_world[i].X;
						pt.y = this->_real_world[i].Y;
						pt.z = this->_real_world[i].Z;
						this->_pcl_cloud.push_back(pt);
					}
				}
			}
		case 1: //JUST FLOOR_PLANE
			{
				uchar* ptr_floor = this->_floor->_mask.data;

				for(int i = 0 ; i < _n_points ; ++i){
					int xx = this->_back_3d_to_2d[i][XX];
					int yy = this->_back_3d_to_2d[i][YY];

					if(ptr[yy * XN_VGA_X_RES + xx] && ptr_floor[yy * XN_VGA_X_RES + xx]){
						//pcl::PointXYZRGB pt(0,0,255);
						pcl::PointXYZRGB pt(ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 2],
											ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 1],
											ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 0]);
						
						pt.x = this->_real_world[i].X;
						pt.y = this->_real_world[i].Y;
						pt.z = this->_real_world[i].Z;
						this->_pcl_cloud.push_back(pt);
					}
				}
			}
			break;
		case 2: //JUST MIRRORS
			{

			}
			break;
		case 3: //FLOOR AND MIRRORS
			{
				uchar* ptr_floor = this->_floor->_mask.data;
				uchar* ptr_mirror = this->_mask_mirrors.data;

				if(this->_floor->_color){
					for(int i = 0 ; i < _n_points ; ++i){
						int xx = this->_back_3d_to_2d[i][XX];
						int yy = this->_back_3d_to_2d[i][YY];

						if(ptr[yy * XN_VGA_X_RES + xx] && ptr_floor[yy * XN_VGA_X_RES + xx]){
							//pcl::PointXYZRGB pt(0,0,255);
							pcl::PointXYZRGB pt(this->_floor->_color_r,this->_floor->_color_g,this->_floor->_color_b);
						
							pt.x = this->_real_world[i].X;
							pt.y = this->_real_world[i].Y;
							pt.z = this->_real_world[i].Z;
							this->_pcl_cloud.push_back(pt);
						}
					}
				}
				else{
					for(int i = 0 ; i < _n_points ; ++i){
						int xx = this->_back_3d_to_2d[i][XX];
						int yy = this->_back_3d_to_2d[i][YY];

						if(ptr[yy * XN_VGA_X_RES + xx] && ptr_floor[yy * XN_VGA_X_RES + xx]){
							//pcl::PointXYZRGB pt(0,0,255);
							pcl::PointXYZRGB pt(ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 2],
												ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 1],
												ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 0]);
						
							pt.x = this->_real_world[i].X;
							pt.y = this->_real_world[i].Y;
							pt.z = this->_real_world[i].Z;
							this->_pcl_cloud.push_back(pt);
						}
					}
				}

				for(int i = 0 ; i < this->_mirrors.size() ; ++i){
					Mirror *mirror = this->_mirrors[i];

					if(mirror){
						if(mirror->_color){
							for(int j = 0 ; j < mirror->_n_points ; ++j){
								int idx = mirror->_points_idx[j];
							
								pcl::PointXYZRGB pt2(mirror->_color_r,mirror->_color_g,mirror->_color_b);
								pt2.x = mirror->_points_mirrored[j].X;
								pt2.y = mirror->_points_mirrored[j].Y;
								pt2.z = mirror->_points_mirrored[j].Z;

								this->_pcl_cloud.push_back(pt2);
							}
						}
						else{
							for(int j = 0 ; j < mirror->_n_points ; ++j){
								int idx = mirror->_points_idx[j];
								//pcl::PointXYZRGB pt(255,255,0);
								//pt.x = mirror->_points[j].X;
								//pt.y = mirror->_points[j].Y;
								//pt.z = mirror->_points[j].Z;

								//this->_pcl_cloud.push_back(pt);
								int xx = this->_back_3d_to_2d[idx][XX];
								int yy = this->_back_3d_to_2d[idx][YY];
								//ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 2] = 0;
								//ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 1] = 0;
								//ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 0] = 255;
								pcl::PointXYZRGB pt2(ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 2],
													 ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 1],
													 ptr_clr[yy * XN_VGA_X_RES * 3 + xx* 3 + 0]);
							
								//pcl::PointXYZRGB pt2(255,255,255);
								pt2.x = mirror->_points_mirrored[j].X;
								pt2.y = mirror->_points_mirrored[j].Y;
								pt2.z = mirror->_points_mirrored[j].Z;

								this->_pcl_cloud.push_back(pt2);
							}
						}
					}
				}

				printf("");
			}
			break;

		default: break;
	}
}

//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
void Controller::remove_floor(){
	uchar* ptr = this->_mask_main.data;
	uchar* ptr_floor = this->_floor->_mask.data;
	for(int i = 0 ; i < this->_n_points ; ++i){
		XnPoint3D pt = this->_real_world[i];
		if(pt.Z > 0.0){
			int xx = this->_back_3d_to_2d[i][XX];
			int yy = this->_back_3d_to_2d[i][YY];

			if(ptr_floor[yy * XN_VGA_X_RES + xx]){
				double dist = this->_floor->_plane.distance_to_plane(pt.X,pt.Y,pt.Z);
				if(dist < this->_floor->_thresh){
					ptr[yy * XN_VGA_X_RES + xx] = 0;
				}
			}
		}
	}
}

void Controller::remove_floor_from_mirrors(){
	for(int i = 0 ; i < this->_mirrors.size() ; ++i){
		Mirror* mirror = this->_mirrors[i];

		for(int k = 0; k < mirror->_n_points ; ++k){
			XnPoint3D pt = mirror->_points_mirrored[k];
			double dist = this->_floor->_plane.distance_to_plane(pt.X,pt.Y,pt.Z);
			if(dist < this->_floor->_thresh){
				
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
			this->_kinect->get_depth(_xn_depth_mat);
		this->_mutex_kinect.unlock();




		UINT16* ptr_16u = (UINT16*)this->_xn_depth_mat.data;
		int n_points = 0;
		for(int y = 0; y < XN_VGA_Y_RES ; y += this->_property_manager->_3d_step) { 
			for(int x = 0; x < XN_VGA_X_RES ; x += this->_property_manager->_3d_step) { 
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = ptr_16u[y * XN_VGA_X_RES + x];//this->_xn_depth_md[y * XN_VGA_X_RES + x]; 

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
	pcl::PolygonMesh polygon;

	this->_3d_viewer = new Viewer3D();

	//this->_3d_viewer->init(&cloud);

	while(_property_manager->_running){
		//Wait for PointCloud
		{
			boost::mutex::scoped_lock lock(this->_mutex_condition_consumer);
			this->_condition_consumer.wait(lock);
		}

		if(this->_property_manager->_flag_processed[PropertyManager::P_POLYGON]){
			if(this->_polygon_ready_show){
				//polygon copy
				polygon = this->_polygon;

				if(polygon.polygons.size()){
					printf("New Polygon\n");
					this->_3d_viewer->show_polygon(&polygon);
				}
				
				this->_polygon_ready_show = false;
			}
			else{
				this->_3d_viewer->spin();
			}
		}
		else{
			if(this->_property_manager->_flag_processed[PropertyManager::P_CAPTURE_SHOW]){
				cloud = this->_model_cloud;

				this->_3d_viewer->show_cloud(&cloud);
			}
			else{
				this->_mutex_pcl.lock();
					cloud = this->_pcl_cloud;
				this->_mutex_pcl.unlock();

				this->_3d_viewer->show_cloud(&cloud);
			}

		}
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

void Controller::thread_pcl_polygon(){
	while(this->_property_manager->_running){
		//Wait for PointCloud
		{
			boost::mutex::scoped_lock lock(this->_mutex_condition_polygon);
			this->_condition_polygon.wait(lock);
		}

		this->generate_polygon();
		
		this->_polygon_ready_show = true;
		this->_polygon_ready_new = true;

		++_frame_counter[Controller::PCL_POLYGON];
		if (_frame_counter[Controller::PCL_POLYGON] == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate[Controller::PCL_POLYGON] = _frame_counter[Controller::PCL_POLYGON] / ((current_tick - _last_tick[Controller::PCL_POLYGON])/cv::getTickFrequency());
			_last_tick[Controller::PCL_POLYGON] = current_tick;
			_frame_counter[Controller::PCL_POLYGON] = 0;
			//printf("\t\t\t PCLProducer FrameRate %.2f\n",_frame_rate);
		}
	}
}

void Controller::thread_gui(int argc, char* argv[]){
	this->_gui->run(argc, argv);
}