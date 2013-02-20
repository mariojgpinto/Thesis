/**
 * Author: Mario Pinto
 *
 * 
 */

#include "Mirror.h"

//-----------------------------------------------------------------------------
// CONTRUCTORS
//-----------------------------------------------------------------------------
Mirror::Mirror():_mask(480,640,(const uchar)0){
	this->setup_variables();
}


Mirror::~Mirror(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/*
 *
 */
void Mirror::setup_variables(){
	for(int i = 0 ; i < _n_flags ; i++)
		this->_flags[i] = false;

	this->_points = new std::vector<cv::Point*>();
	this->_user_points = new std::vector<cv::Point*>();
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
inline void Mirror::enable_flag(Mirror::FLAGS flag, bool value){
	this->_flags[flag] = value;
}

/**
 *
 *
 * @param flag
 *
 *
 */
inline bool Mirror::check_flag(Mirror::FLAGS flag){
	return this->_flags[flag];
}

//-----------------------------------------------------------------------------
// MIRROR CONSTRUCTION
//-----------------------------------------------------------------------------
/**
 *
 *
 * @param points
 *
 *
 */
void Mirror::set_area(std::vector<cv::Point*>* points){
	if(!points && points->size() > 2) return;

	cv::Point *pointsi = (cv::Point*)malloc(sizeof(cv::Point));
	for(unsigned int i = 0 ; i < points->size() ; i++){
		if(points->at(i)){
			cv::Point* point = points->at(i);
			if(ToolBoxCV::in_range(point)){
				pointsi[i] = cv::Point(*points->at(i));
			}
		}
	}
	
	const cv::Point* countours[1]={
        pointsi,
    };

	const int countours_n[1]={
        points->size(),      
    };

	cv::Mat1b mask(480,640,(const uchar)0);
	bool ok = true;
	try{
		cv::fillPoly(mask,countours,countours_n,1,cv::Scalar(255,255,255));
	}
	catch(...){
		printf("");
		ok = false;
	}
	
	if(ok){
		this->_points->clear();
		for(unsigned int i = 0 ; i < points->size() ; i++){
			if(points->at(i)){
				this->_points->push_back(new cv::Point(*points->at(i)));
			}
		}

		mask.assignTo(this->_mask);
		this->_flags[Mirror::MASK] = true;
	}
}

/**
 *
 *
 * @param a
 *
 *
 * @param b
 *
 *
 * @param c
 *
 *
 * @param d
 *
 *
 */
void Mirror::set_plane(int a, int b, int c, int d){
	this->_plane.a = a;
	this->_plane.b = b;
	this->_plane.c = c;
	this->_plane.d = d;
	
	this->_flags[Mirror::PLANE] = true;
}

/**
 *
 *
 * @param index
 *
 *
 * @param point
 *
 *
 */
void Mirror::update_vertex(int index, cv::Point* point){
	if(point && index >= 0 && index < (int)this->_points->size()){
		this->_points->at(index)->x = point->x;
		this->_points->at(index)->y = point->y;
		
		std::vector<cv::Point*> aux;

		for(unsigned int i = 0 ; i < this->_points->size() ; i++){
			aux.push_back(new cv::Point(*this->_points->at(i)));
		}

		this->set_area(&aux);
	}
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
cv::Mat1b* Mirror::get_mask(){
	return (this->_flags[Mirror::MASK]) ? &this->_mask : NULL;
}

/**
 *
 *
 * @return
 *
 *
 */
inline ntk::Plane* Mirror::get_plane(){
	return (this->_flags[Mirror::PLANE]) ? &this->_plane : NULL;
}



/**
 *
 *
 * @return
 *
 *
 */
 bool Mirror::is_valid(){
	return true;
}


//-----------------------------------------------------------------------------
// ACCESS - MIRROR MANAGEMENT
//-----------------------------------------------------------------------------
/** 
 *
 *
 * @return
 *
 *
 */
int Mirror::get_n_vertexes(){
	return this->_points->size();
}

/** 
 *
 *
 * @return
 *
 *
 */
std::vector<cv::Point*>* Mirror::get_vertexes(){
	return this->_points;
}

/** 
 *
 *
 * @param index
 *
 *
 * @return
 *
 *
 */
cv::Point* Mirror::get_vertex(int index){
	if(index >= 0 && index < (int)this->_points->size()){
		return this->_points->at(index);
	}
	return NULL;
}