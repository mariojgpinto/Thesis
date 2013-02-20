/**
 * Author: Mario Pinto
 *
 * 
 */

#include "Arena.h"

//-----------------------------------------------------------------------------
// CONTRUCTORS
//-----------------------------------------------------------------------------
Arena::Arena(){
	this->setup_variables();
}

Arena::~Arena(){

}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 *
 */
void Arena::setup_variables(){
	for(int i = 0 ; i < Arena::_n_flags ; i++)
		this->_flags[i] = false;

	this->_mirrors = new std::vector<Mirror*>();

	this->_floor = NULL;
}

/**
 *
 *
 * @param flag 
 *
 *
 * @param value
 *
 *
 */
inline void Arena::enable_flag(Arena::FLAGS flag, bool value){
	this->_flags[flag] = value;
}

/**
 *
 *
 * @param flag 
 *
 *
 */
inline bool Arena::check_flag(Arena::FLAGS flag){
	return this->_flags[flag];
}

//-----------------------------------------------------------------------------
// CONFIGURATION
//-----------------------------------------------------------------------------
/**
 *
 *
 * @param mirror
 *
 *
 */
void Arena::add_mirror(Mirror* mirror){
	if(mirror && mirror->is_valid()){
		this->_mirrors->push_back(mirror);
		this->_flags[Arena::MIRRORS] = true;
	}
}

/**
 *
 *
 * @param mirror
 *
 *
 */
void Arena::add_floor(Floor* floor){
	if(floor){
		this->_flags[Arena::FLOOR] = true;
	}
}

//-----------------------------------------------------------------------------
// UTIL
//-----------------------------------------------------------------------------

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
/**
 *
 *
 * @param points
 *
 *
 * @param image
 *
 *
 * @param point_radius
 *
 *
 * @return 
 *
 *
 */
ntk::Plane* Arena::extract_plane(std::vector<cv::Point*>* points, ntk::RGBDImage* image, int point_radius){
	if(!points || !image) return NULL;

	std::string* pcl_file = new std::string("file.pcd");

	FILE* fp = NULL;
	fopen_s(&fp, pcl_file->data(),"w+");

	if(!fp) return NULL;

	fprintf(fp,"# .PCD v0.7 - Point Cloud Data file format\n");
	fprintf(fp,"VERSION 0.7\n");
	fprintf(fp,"FIELDS x y z\n");
	fprintf(fp,"SIZE 4 4 4\n");
	fprintf(fp,"TYPE F F F\n");
	fprintf(fp,"COUNT 1 1 1\n");
	fprintf(fp,"WIDTH %d\n", points->size() * 10 * 10);
	fprintf(fp,"HEIGHT 1\n");
	fprintf(fp,"VIEWPOINT 0 0 0 1 0 0 0\n");
	fprintf(fp,"POINTS %d\n", points->size() * point_radius * point_radius);
	fprintf(fp,"DATA ascii\n");

	//TODO Change writting to file to cloud input
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	

	cv::Point pt1;
	cv::Point3f pf1;
	for(int i = -point_radius ; i < point_radius ; i++){
		for(int j = -5 ; j < 5 ; j++){
			for(int k = 0 ; k < (int)points->size() ; k++){
				pt1.x = points->at(k)->x + i;
				pt1.y = points->at(k)->y + j;

				pf1 = image->calibration()->depth_pose->unprojectFromImage(pt1, image->depth()(pt1));

				if(pf1.x == 0 && pf1.y == 0 && pf1.z == 0) fprintf(fp,"nan nan nan\n");
				else{
					fprintf(fp,"%f %f %f\n", pf1.x, pf1.y,pf1.z);
					//cloud->push_back(pcl::PointXYZ(pf1.x,pf1.y,pf1.z));
					//cloud->points.push_back(pcl::PointXYZ(pf1.x,pf1.y,pf1.z));
				}
			}
		}
	}

	
				

	fclose(fp);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcl_file->data(), *cloud) == -1) //* load the file
	{
		//PCL_ERROR ("Couldn't read file floor.pcd \n");
		return NULL;
	}

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.02);

	seg.setInputCloud (cloud->makeShared ());
	seg.segment (*inliers, *coefficients);

	coefficients->values.size();

	ntk::Plane* plane = new ntk::Plane(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
	
	return plane;
}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
/**
 *
 *
 * @return
 *
 *
 */
std::vector<Mirror*>* Arena::get_mirrors(){
	return this->_mirrors;
}

/**
 *
 *
 * @param mirror
 *
 *
 * @return 
 *
 *
 */
Mirror* Arena::get_mirror(int index){
	if(index >= 0 && index < (int)this->_mirrors->size())
		return this->_mirrors->at(index);
	return NULL;
}

/**
 *
 *
 * @return 
 *
 *
 */
Floor* Arena::get_floor(){
	return this->_floor;
}

/**
 *
 *
 * @return 
 *
 *
 */
int Arena::get_n_mirrors(){
	return this->_mirrors->size();
}