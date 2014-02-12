#include "PropertyManager.h"

PropertyManager::PropertyManager(){
	this->_running = true;
	this->_pause = false;
	this->_3d_step = 2;
	this->_depth_min = 500;
	this->_depth_max = 5000;

	this->_outliers_methods = 1;

	this->_voxel_grid_x = 1;
	this->_voxel_grid_y = 1;
	this->_voxel_grid_z = 1;

	this->_outliers_radius_neighbors = 2;
	this->_outliers_radius_radius = 0.8;
	this->_outliers_statistical_meank = 25;
	this->_outliers_statistical_stddev = 0.5;

	for(int i = 0 ; i < this->_n_request_flags ; ++i){
		this->_flag_requests[i] = false;
	}

	for(int i = 0 ; i < this->_n_processed_flags; ++i){
		this->_flag_processed[i] = false;
	}

	for(int i = 0 ; i < this->_n_update_flags; ++i){
		this->_flag_update[i] = true;
	}
}

PropertyManager::~PropertyManager(){

}