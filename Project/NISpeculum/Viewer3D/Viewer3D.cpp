/** 
 * @file	Viewer3D.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	July, 2013 
 * @brief	Implementation of the Viewer3D Class.
 *
 * @details	Detailed Information.
 */
#include "Viewer3D.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------

/**
 * @brief	Default constructor for the 3DViewer
 *
 */
Viewer3D::Viewer3D(char* window_name){
	this->_last_tick = 0;
	this->_frame_counter = 0;
	this->_frame_rate = 0;

	this->_viewer = (window_name) ? new pcl::visualization::CloudViewer(window_name):
									new pcl::visualization::CloudViewer("Viewer3D");
}

Viewer3D::~Viewer3D(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void Viewer3D::init(){

}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
void Viewer3D::show_cloud(pcl::PointCloud<pcl::PointXYZ>* cloud){
	if(cloud)
		this->_viewer->showCloud(cloud->makeShared());
}

void Viewer3D::show_cloud(pcl::PointCloud<pcl::PointXYZRGB>* cloud){
	if(cloud)
		this->_viewer->showCloud(cloud->makeShared());
}