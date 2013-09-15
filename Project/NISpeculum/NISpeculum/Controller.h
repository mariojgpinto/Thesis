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

//Explanation: http://www.pcl-users.org/Eigen-Dense-Storage-Assertion-Error-Solved-td4023763.html
//#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DONT_VECTORIZE 
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

class PropertyManager;
class GUIController;
class NIKinect;
class Viewer3D;
class Floor;
class Mirror;

#include <opencv2\opencv.hpp>
#include <XnCppWrapper.h>
#include <boost\thread.hpp>
#include <pcl\point_types.h>
#include <pcl\PolygonMesh.h>

#define XX 0
#define YY 1
#define ZZ 2

class Controller{
	public:
		enum THREAD{
			CONTROLLER,
			KINECT,
			GUI,
			PCL_PRODUCER,
			PCL_CONSUMER,
			PCL_POLYGON
		};
		static const int _n_thread_flags = 6;


	public:
		Controller();
		~Controller();

		//Setup
		void init(const char* oni_path = 0);

		//Running
		void run(int argc, char* argv[]);

	public:
		//Arena
		void construct_mirrors_masks();
		void construct_global_mask();

		//Storage
		bool save_to_file(char* path = 0, char* filename = 0);
		bool load_from_file(char* file_path);

	private:
		//Thread Functions
		void thread_kinect();
		void thread_gui(int argc, char* argv[]);
		void thread_pcl_producer();
		void thread_pcl_consumer();
		void thread_pcl_polygon();

		//Running
		void process_request();
		void process_images();
		void process_masks();
		void process_mirrors_masks();
		void process_floor_mask();

		bool generate_3d();
		bool generate_3d_mirrors();

		void remove_floor();
		void remove_floor_from_mirrors();

		void add_mirror(Mirror* mirror);
		void add_floor(Floor *floor);

		void model_add_frame();
		void generate_model();

		void generate_polygon();

		void copy_to_pcl(int flag);

	public:
		//MEMBERS
		PropertyManager* _property_manager;
		GUIController* _gui;
		NIKinect* _kinect;
		Viewer3D* _3d_viewer;
		Floor* _floor;
		int _n_mirrors;
		std::vector<Mirror*> _mirrors;

		//VARIABLES
		cv::Mat _mat_color_bgr;
		cv::Mat _mat_color_rgb;
		cv::Mat _mat_depth8UC1;
		cv::Mat _mat_depth16UC1;
		
		cv::Mat _mask_main;
		cv::Mat _mask_depth;
		cv::Mat _mask_floor;
		cv::Mat _mask_mirrors;
		cv::Mat _mask_mirrors_and_foor;
		
		xn::DepthMetaData _xn_depth_md;
		xn::ImageMetaData _xn_color_md;
				
		XnPoint3D* _point_list;// = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
		XnPoint3D* _real_world;// = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
		pcl::PointCloud<pcl::PointXYZRGB> _pcl_cloud;
		bool _polygon_ready_new;
		bool _polygon_ready_show;
		pcl::PointCloud<pcl::PointXYZRGB> _pcl_cloud_polygon;
		pcl::PolygonMesh _polygon;
		boost::mutex _mutex_condition_polygon;
		boost::condition_variable _condition_polygon;
		//pcl::PointCloud<pcl::PointXYZ> _pcl_cloud;

		int _n_points;
		int **_back_3d_to_2d; //X && Y
		int **_back_2d_to_3d; //
		
		int _floor_n_points;
		float **_floor_points;
		int *_back_floor_to_realworld;

		float **_mirror_points;
		int *_back_mirror_to_realworld;


		std::vector<cv::Point*>* _aux_points;

		int _model_n_frames;
		std::vector<cv::Mat> _model_frames_depth;
		std::vector<cv::Mat> _model_frames_color;
		cv::Mat _model_depth_average;
		cv::Mat _model_color_average;
		int _model_n_points;
		XnPoint3D* _model_projective;
		XnPoint3D* _model_realworld;
		pcl::PointCloud<pcl::PointXYZRGB> _model_cloud;
		pcl::PolygonMesh _model_polygon;


	private:		
		//THREADS
		boost::thread *_t_kinect;
		boost::thread *_t_gui;
		boost::thread *_t_pcl_consumer;
		boost::thread *_t_pcl_producer;
		boost::thread *_t_pcl_polygon;

		boost::mutex _mutex_kinect;
		boost::mutex _mutex_pcl;

		boost::mutex _mutex_condition_consumer;
		boost::condition_variable _condition_consumer;

		double  _last_tick[_n_thread_flags];
		int _frame_counter[_n_thread_flags];
		float  _frame_rate[_n_thread_flags];
};

#endif//_CONTROLLER