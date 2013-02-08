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
void Arena::setup_variables(){
	this->_flags = (bool*)malloc(sizeof(bool)* Arena::_n_flags);
	for(int i = 0 ; i < Arena::_n_flags ; i++)
		this->_flags[i] = false;

	this->_mirrors = new std::vector<Mirror*>();

	this->_floor = NULL;
}

inline void Arena::enable_flag(Arena::FLAGS flag, bool value){
	this->_flags[flag] = value;
}

inline bool Arena::check_flag(Arena::FLAGS flag){
	return this->_flags[flag];
}

//-----------------------------------------------------------------------------
// CONFIGURATION
//-----------------------------------------------------------------------------
void Arena::add_mirror(Mirror* mirror){

}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
std::vector<Mirror*>* Arena::get_mirrors(){
	return this->_mirrors;
}

Mirror* Arena::get_mirror(int index){
	if(index > 0 && index < (int)this->_mirrors->size())
		return this->_mirrors->at(index);
	return NULL;
}

Floor* Arena::get_floor(){
	return this->_floor;
}