#include "Floor.h"


Floor::Floor(){

}


Floor::~Floor(){

}



void Floor::setup_variables(){
	for(int i = 0 ; i < _n_flags ; i++)
		this->_flags[i] = false;
}