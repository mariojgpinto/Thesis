#include "PropertyManager.h"

PropertyManager::PropertyManager(){
	this->_running = true;
	this->_pause = false;
	this->_3d_step = 2;
	this->_depth_min = 500;
	this->_depth_max = 1500;

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