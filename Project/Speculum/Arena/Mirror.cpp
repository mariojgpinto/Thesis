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

}


Mirror::~Mirror(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void Mirror::setup_variables(){

	this->_flags = (bool*)malloc(sizeof(bool)*_n_flags);
	for(int i = 0 ; i < _n_flags ; i++)
		this->_flags = false;
}

inline void Mirror::enable_flag(Mirror::FLAGS flag, bool value){
	this->_flags[flag] = value;
}

inline bool Mirror::check_flag(Mirror::FLAGS flag){
	return this->_flags[flag];
}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
// MIRROR CONSTRUCTION
//-----------------------------------------------------------------------------
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
		
		
		//cv::imshow("poly",_mask);
		//cv::waitKey(30);
		//cv::imshow("poly",_mask);
		//cv::waitKey(30);

	}
	catch(...){
		printf("");
		ok = false;
	}
	
	if(ok){
		mask.assignTo(this->_mask);
	}


	//polylines(this->_mask, &pts,&npts, 1,
	//    		true, 			// draw closed contour (i.e. joint end to start) 
	//            cv::Scalar(255,255,255),// colour RGB ordering (here = green) 
	//    		3, 		        // line thickness
	//		    CV_AA, 0);


}