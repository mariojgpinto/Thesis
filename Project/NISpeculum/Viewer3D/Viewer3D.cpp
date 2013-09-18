/** 
 * @file	Viewer3D.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	July, 2013 
 * @brief	Implementation of the Viewer3D Class.
 *
 * @details	Detailed Information.
 */
#include "Viewer3D.h"

#include <pcl/io/pcd_io.h>

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------

/**
 * @brief	Default constructor for the 3DViewer
 *
 */
Viewer3D::Viewer3D(char* window_name)
//_viewer(pcl::visualization::PCLVisualizer("Viewer3D"))
{
	this->_last_tick = 0;
	this->_frame_counter = 0;
	this->_frame_rate = 0;

	 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	 this->_viewer = viewer;
	//this->_viewer = (window_name) ? new pcl::visualization::PCLVisualizer(window_name):
	//								new pcl::visualization::PCLVisualizer("Viewer3D");
	
	this->init();
}

Viewer3D::~Viewer3D(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void Viewer3D::init(){
	//_viewer->setBackgroundColor (0, 0, 0);
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	this->_viewer->addPointCloud(cloud.makeShared(),"cloud");

	_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
	_viewer->addCoordinateSystem (1.0);
	_viewer->initCameraParameters ();
	
}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
void Viewer3D::show_cloud(pcl::PointCloud<pcl::PointXYZ>* cloud){
	if(cloud){
		//this->_viewer->showCloud(cloud->makeShared());
		//this->_viewer->removeAllPointClouds();
		//this->_viewer->addPointCloud(cloud->makeShared(),"cloud");
		this->_viewer->updatePointCloud(cloud->makeShared(),"cloud");
		this->_viewer->spinOnce(50);
	}
}

void Viewer3D::show_cloud(pcl::PointCloud<pcl::PointXYZRGB>* cloud){
	if(cloud){
		//this->_viewer->showCloud(cloud->makeShared());
		//this->_viewer->removeAllPointClouds();
		//this->_viewer->addPointCloud(cloud->makeShared(),"cloud");
		this->_viewer->updatePointCloud(cloud->makeShared(),"cloud");
		this->_viewer->spinOnce(50);
	}
}

void Viewer3D::show_polygon(pcl::PolygonMesh* polygon){
	if(polygon){
		this->_viewer->removeAllShapes();
		this->_viewer->removeAllPointClouds();
		this->_viewer->addPolygonMesh(*polygon);
		//this->_viewer->updatePolygonMesh(*polygon);
		this->_viewer->spinOnce(50);
	}
}

void Viewer3D::spin(){
	this->_viewer->spinOnce(50);
}