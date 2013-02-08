#include "Floor.h"


Floor::Floor(){

}


Floor::~Floor(){

}



void Floor::setup_variables(){
	this->_flags = (bool*)malloc(sizeof(bool)*_n_flags);
	for(int i = 0 ; i < _n_flags ; i++)
		this->_flags = false;
}