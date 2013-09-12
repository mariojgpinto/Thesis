/** 
 * @file	Viewer3D.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	July, 2013 
 * @brief	Declaration of the Viewer3D Class.
 *
 * @details	Detailed Information.
 */
#ifndef _VIEWER_3D
#define _VIEWER_3D

#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

class __declspec(dllexport) Viewer3D{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	public:
		Viewer3D(char* window_name = 0);
		~Viewer3D();

		void init();

		void spin();

		void show_cloud(pcl::PointCloud<pcl::PointXYZ>* cloud);
		void show_cloud(pcl::PointCloud<pcl::PointXYZRGB>* cloud);

		void show_polygon(pcl::PolygonMesh* polygon);

	private:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;

		double _last_tick;
		int	_frame_counter;
		float _frame_rate;
};

#endif//_VIEWER_3D