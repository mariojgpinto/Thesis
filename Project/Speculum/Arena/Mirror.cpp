/**
 * Author: Mario Pinto
 *
 * 
 */

#include "Mirror.h"

//-----------------------------------------------------------------------------
// CONTRUCTORS
//-----------------------------------------------------------------------------
Mirror::Mirror():_area_mask(480,640,(const uchar)0){
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
	
	this->_area_max_width = INT_MIN;
	this->_area_max_height = INT_MIN;
	this->_area_min_width = INT_MAX;
	this->_area_min_height = INT_MAX;
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
				this->area_max_min(points->at(i));
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

		mask.assignTo(this->_area_mask);
		this->_flags[Mirror::MASK] = true;
		this->_flags[Mirror::INPUT] = true;
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
void Mirror::set_plane(double a, double b, double c, double d){
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
// MIRROR CONSTRUCTION
//-----------------------------------------------------------------------------
/**
 * @brief
 * @details
 *
 * @param pt
 *
 */
void Mirror::area_max_min(cv::Point* pt){
	if(pt->x < this->_area_min_width)
		this->_area_min_width = pt->x;

	if(pt->y < this->_area_min_height)
		this->_area_min_height = pt->y;

	if(pt->x > this->_area_max_width)
		this->_area_max_width = pt->x;
	
	if(pt->y > this->_area_max_height)
		this->_area_max_height = pt->y;
}

//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
void Mirror::add_perspective_to_mesh(ntk::Mesh *mesh, ntk::RGBDImage* image){
	if(!mesh || !image) return;

	if(!this->_flags[Mirror::PLANE]){
		return ;
	}

	cv::Mat1f aux_depth = image->depth();
	cv::Mat1b aux_depth_mask = image->depthMask();
	cv::Mat3b aux_color = image->rgb();
	ntk::Pose3D* aux_pose = image->calibration()->depth_pose;
	
	//Create a mask with all existing masks
	cv::Mat1b mega_mask;
	image->depthMask().copyTo(mega_mask);

	//image max and min to 
	int max_width;
	int max_height;
	int min_width;
	int min_height;

	if(this->_flags[Mirror::MASK]){
		max_width = this->_area_max_width;
		max_height = this->_area_max_height;
		min_width = this->_area_min_width;
		min_height = this->_area_min_height;

		cv::bitwise_and(mega_mask,this->_area_mask,mega_mask);
	}
	else{
		max_width = image->depth().cols;
		max_height = image->depth().rows;
		min_width = 0;
		min_height = 0;
	}

	cv::Vec3f normal = this->_plane.normal();
	normal*=-1;

	cv::Point3f middle(0.012746937,0.12180407,-0.75500000);

	for(int i = min_width ; i < max_width ; i++){
		for(int j = min_height ; j < max_height ; j++){
			cv::Point p(i,j);
			//If the point is part of the mask
			float coiso = aux_depth(p);

			if(mega_mask(p) && coiso < 1){
				//Unprojected point
				cv::Point3f pf = aux_pose->unprojectFromImage(p, aux_depth(p));
				
				

				//Project from Mirror
				double dp = this->_plane.distanceToPlane(pf);

				if(dp < 0.75){
					cv::Point3f pt2;
					pt2.x = pf.x + (dp * 2 * normal.val[0]);
					pt2.y = pf.y + (dp * 2 * normal.val[1]);
					pt2.z = pf.z + (dp * 2 * normal.val[2]);

					//Color vector to pass from RGB to BGR
					cv::Vec3b color = aux_color(p);

					//Create the Surfel, add porperties and add to mesh
					ntk::Surfel surf;
					surf.location = cv::Point3f(pt2);
					surf.color = cv::Vec3f(color.val[2],color.val[1],color.val[0]);
					mesh->addPointFromSurfel(surf);
				}
			}
		}
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
cv::Mat1b* Mirror::get_area_mask(){
	return (this->_flags[Mirror::MASK]) ? &this->_area_mask : NULL;
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
	return this->_flags[Mirror::PLANE];
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

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
#include "ArenaXML.h"

bool Mirror::save_to_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *elem, std::string* path, int index){
	if(!doc || !path) return false;

	bool result = false;

	//FLAGS
	{
		tinyxml2::XMLElement *elem_flags = doc->NewElement(_XML_MIRROR_ELEM_FLAGS);

			//PLANE FLAG
			tinyxml2::XMLElement *elem_flag_plane = doc->NewElement(_XML_MIRROR_ELEM_FLAG_PLANE);
			elem_flag_plane->SetAttribute(_XML_FLAG,this->_flags[Mirror::PLANE]);
			elem_flags->LinkEndChild(elem_flag_plane);

			//MASK FLAG
			tinyxml2::XMLElement *elem_flag_mask = doc->NewElement(_XML_MIRROR_ELEM_FLAG_MASK);
			elem_flag_mask->SetAttribute(_XML_FLAG,this->_flags[Mirror::MASK]);
			elem_flags->LinkEndChild(elem_flag_mask);

			//INPUT FLAG
			tinyxml2::XMLElement *elem_flag_input = doc->NewElement(_XML_MIRROR_ELEM_FLAG_INPUT);
			elem_flag_input->SetAttribute(_XML_FLAG,this->_flags[Mirror::INPUT]);
			elem_flags->LinkEndChild(elem_flag_input);

		elem->LinkEndChild(elem_flags);
	}

	//PLANE
	if(this->_flags[Mirror::PLANE])
	{
		tinyxml2::XMLElement *elem_plane = doc->NewElement(_XML_MIRROR_ELEM_PLANE);
			
			tinyxml2::XMLElement *elem_plane_a = doc->NewElement(_XML_MIRROR_ELEM_PLANE_A);
			elem_plane_a->SetAttribute(_XML_VALUE,this->_plane.a);
			elem_plane->LinkEndChild(elem_plane_a);

			tinyxml2::XMLElement *elem_plane_b = doc->NewElement(_XML_MIRROR_ELEM_PLANE_B);
			elem_plane_b->SetAttribute(_XML_VALUE,this->_plane.b);
			elem_plane->LinkEndChild(elem_plane_b);

			tinyxml2::XMLElement *elem_plane_c = doc->NewElement(_XML_MIRROR_ELEM_PLANE_C);
			elem_plane_c->SetAttribute(_XML_VALUE,this->_plane.c);
			elem_plane->LinkEndChild(elem_plane_c);

			tinyxml2::XMLElement *elem_plane_d = doc->NewElement(_XML_MIRROR_ELEM_PLANE_D);
			elem_plane_d->SetAttribute(_XML_VALUE,this->_plane.d);
			elem_plane->LinkEndChild(elem_plane_d);

		elem->LinkEndChild(elem_plane);
	}

	//AREA MASK
	if(this->_flags[Mirror::MASK])
	{
		tinyxml2::XMLElement *elem_mask = doc->NewElement(_XML_MIRROR_ELEM_AREA);
			
			tinyxml2::XMLElement *elem_mask_mask = doc->NewElement(_XML_MIRROR_ELEM_AREA_MASK);
			
			std::string image_file_name;
			image_file_name.assign(_XML_MIRROR_FILE_AREA_MASK);
			char buff[8];
			itoa(index,buff,10);
			image_file_name.append(buff);
			image_file_name.append(_XML_MIRROR_FILE_AREA_MASK_PNG);

			result = ToolBoxXML::cv_save_image(elem_mask,elem_mask_mask,(char*)path->data(),(char*)image_file_name.data(),this->_area_mask);
			if(!result) return false;

			tinyxml2::XMLElement *elem_area_limits = doc->NewElement(_XML_MIRROR_ELEM_AREA_LIMITS);
			
				tinyxml2::XMLElement *elem_area_limits_max_w = doc->NewElement(_XML_MIRROR_ELEM_AREA_MAX_WIDTH);
				elem_area_limits_max_w->SetAttribute(_XML_VALUE,this->_area_max_width);
				elem_area_limits->LinkEndChild(elem_area_limits_max_w);

				tinyxml2::XMLElement *elem_area_limits_min_w = doc->NewElement(_XML_MIRROR_ELEM_AREA_MIN_WIDTH);
				elem_area_limits_min_w->SetAttribute(_XML_VALUE,this->_area_min_width);
				elem_area_limits->LinkEndChild(elem_area_limits_min_w);

				tinyxml2::XMLElement *elem_area_limits_max_h = doc->NewElement(_XML_MIRROR_ELEM_AREA_MAX_HEIGHT);
				elem_area_limits_max_h->SetAttribute(_XML_VALUE,this->_area_max_height);
				elem_area_limits->LinkEndChild(elem_area_limits_max_h);

				tinyxml2::XMLElement *elem_area_limits_min_h = doc->NewElement(_XML_MIRROR_ELEM_AREA_MIN_HEIGHT);
				elem_area_limits_min_h->SetAttribute(_XML_VALUE,this->_area_min_height);
				elem_area_limits->LinkEndChild(elem_area_limits_min_h);

			elem_mask->LinkEndChild(elem_area_limits);

		elem->LinkEndChild(elem_mask);
	}

	//INPUT
	if(this->_flags[Mirror::INPUT])
	{
		tinyxml2::XMLElement *elem_input = doc->NewElement(_XML_MIRROR_ELEM_INPUT);
			
			tinyxml2::XMLElement *elem_input_n = doc->NewElement(_XML_MIRROR_ELEM_INPUT_N);
			elem_input_n->SetAttribute(_XML_VALUE,this->_points->size());
			elem_input->LinkEndChild(elem_input_n);
			
			tinyxml2::XMLElement *elem_input_vertexes = doc->NewElement(_XML_MIRROR_ELEM_INPUT_POINTS);
			
			for(unsigned int i = 0 ; i < this->_points->size() ; i++){
				tinyxml2::XMLElement *elem_input_vertex = doc->NewElement(_XML_MIRROR_ELEM_INPUT_POINT);
				result = ToolBoxXML::cv_add_point(elem_input_vertex,this->_points->at(i));
				
				if(!result) return false;
				
				elem_input_vertexes->LinkEndChild(elem_input_vertex);
			}

			elem_input->LinkEndChild(elem_input_vertexes);


		elem->LinkEndChild(elem_input);
	}

	return true;
}

bool Mirror::load_from_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *root){
	if(!doc || !root) return false;

	tinyxml2::XMLError error;

	//READ FLAGS
	{
		tinyxml2::XMLElement *elem_flags = root->FirstChildElement(_XML_MIRROR_ELEM_FLAGS);

		if(!elem_flags) return false;

		//PLANE FLAG
		tinyxml2::XMLElement *elem_flag_plane = elem_flags->FirstChildElement(_XML_MIRROR_ELEM_FLAG_PLANE);

		if(elem_flag_plane){
			bool temp_flag = false;
			error = elem_flag_plane->QueryBoolAttribute(_XML_FLAG,&temp_flag);
			if(error) this->enable_flag(Mirror::PLANE,false);
			else this->enable_flag(Mirror::PLANE,temp_flag);
		} else this->enable_flag(Mirror::PLANE,false);

		//MASK FLAG
		tinyxml2::XMLElement *elem_flag_mask = elem_flags->FirstChildElement(_XML_MIRROR_ELEM_FLAG_MASK);

		if(elem_flag_mask){
			bool temp_flag = false;
			error = elem_flag_mask->QueryBoolAttribute(_XML_FLAG,&temp_flag);
			if(error) this->enable_flag(Mirror::MASK,false);
			else this->enable_flag(Mirror::MASK,temp_flag);
		} else this->enable_flag(Mirror::MASK,false);

		//INPUT FLAG
		tinyxml2::XMLElement *elem_flag_input = elem_flags->FirstChildElement(_XML_MIRROR_ELEM_FLAG_INPUT);

		if(elem_flag_input){
			bool temp_flag = false;
			error = elem_flag_input->QueryBoolAttribute(_XML_FLAG,&temp_flag);
			if(error) this->enable_flag(Mirror::INPUT,false);
			else this->enable_flag(Mirror::INPUT,temp_flag);
		} else this->enable_flag(Mirror::INPUT,false);
	}

	//READ PLANE
	if(this->_flags[Mirror::PLANE]){
		tinyxml2::XMLElement *elem_plane = root->FirstChildElement(_XML_MIRROR_ELEM_PLANE);

		if(!elem_plane) return false;

		double a,b,c,d;

		//READ 'A' VALUE
		tinyxml2::XMLElement *elem_floor_plane_a = elem_plane->FirstChildElement(_XML_MIRROR_ELEM_PLANE_A);
		if(elem_floor_plane_a){
			error = elem_floor_plane_a->QueryDoubleAttribute(_XML_VALUE,&a);
			if(error)
				return false;
		}
		//READ 'B' VALUE
		tinyxml2::XMLElement *elem_floor_plane_b = elem_plane->FirstChildElement(_XML_MIRROR_ELEM_PLANE_B);
		if(elem_floor_plane_b){
			error = elem_floor_plane_b->QueryDoubleAttribute(_XML_VALUE,&b);
			if(error)
				return  false;
		}
		//READ 'C' VALUE
		tinyxml2::XMLElement *elem_floor_plane_c = elem_plane->FirstChildElement(_XML_MIRROR_ELEM_PLANE_C);
		if(elem_floor_plane_c){
			error = elem_floor_plane_c->QueryDoubleAttribute(_XML_VALUE,&c);
			if(error)
				return false;
		}
		//READ 'D' VALUE
		tinyxml2::XMLElement *elem_floor_plane_d = elem_plane->FirstChildElement(_XML_MIRROR_ELEM_PLANE_D);
		if(elem_floor_plane_d){
			error = elem_floor_plane_d->QueryDoubleAttribute(_XML_VALUE,&d);
			if(error)
				return  false;
		}

		if(this->_flags[Mirror::PLANE]){
			this->_plane.a = a;
			this->_plane.b = b;
			this->_plane.c = c;
			this->_plane.d = d;
		}
	}

	//READ MASK
	if(this->_flags[Mirror::MASK]){
		tinyxml2::XMLElement *elem_mask = root->FirstChildElement(_XML_MIRROR_ELEM_AREA);

		if(!elem_mask) return false;

		bool result = ToolBoxXML::cv_load_image_xml(elem_mask,_XML_MIRROR_ELEM_AREA_MASK,this->_area_mask);
		if(!result) 
			return false;

		tinyxml2::XMLElement *elem_mask_limits = elem_mask->FirstChildElement(_XML_MIRROR_ELEM_AREA_LIMITS);

		if(!elem_mask_limits) return false;

		tinyxml2::XMLElement *elem_area_limits_max_w = elem_mask_limits->FirstChildElement(_XML_MIRROR_ELEM_AREA_MAX_WIDTH);
		if(elem_area_limits_max_w){
			error = elem_area_limits_max_w->QueryIntAttribute(_XML_VALUE,&this->_area_max_width);
			if(error) return false;
		}else return false;

		tinyxml2::XMLElement *elem_area_limits_min_w = elem_mask_limits->FirstChildElement(_XML_MIRROR_ELEM_AREA_MIN_WIDTH);
		if(elem_area_limits_min_w){
			error = elem_area_limits_min_w->QueryIntAttribute(_XML_VALUE,&this->_area_min_width);
			if(error) return false;
		}else return false;

		tinyxml2::XMLElement *elem_area_limits_max_h = elem_mask_limits->FirstChildElement(_XML_MIRROR_ELEM_AREA_MAX_HEIGHT);
		if(elem_area_limits_max_h){
			error = elem_area_limits_max_h->QueryIntAttribute(_XML_VALUE,&this->_area_max_height);
			if(error) return false;
		}else return false;

		tinyxml2::XMLElement *elem_area_limits_min_h = elem_mask_limits->FirstChildElement(_XML_MIRROR_ELEM_AREA_MIN_HEIGHT);
		if(elem_area_limits_min_h){
			error = elem_area_limits_min_h->QueryIntAttribute(_XML_VALUE,&this->_area_min_height);
			if(error) return false;
		}else return false;
	}

	//READ INPUT
	if(this->_flags[Mirror::INPUT]){
		tinyxml2::XMLElement *elem_input = root->FirstChildElement(_XML_MIRROR_ELEM_INPUT);

		if(!elem_input) return false;

		int n_points = 0;
		tinyxml2::XMLElement *elem_input_n_pts = elem_input->FirstChildElement(_XML_MIRROR_ELEM_INPUT_N);
		if(!elem_input_n_pts) return false;

		error = elem_input_n_pts->QueryIntAttribute(_XML_VALUE,&n_points);
		if(error) return false;

		tinyxml2::XMLElement *elem_input_pts = elem_input->FirstChildElement(_XML_MIRROR_ELEM_INPUT_POINTS);
		if(!elem_input_pts) return false;

		tinyxml2::XMLElement *elem_input_pt = elem_input_pts->FirstChildElement(_XML_FLOOR_ELEM_INPUT_POINT);

		for(int i = 0 ; i < n_points ; i++){
			if(!elem_input_pt) return false;

			cv::Point pt;
			bool result = ToolBoxXML::cv_read_point(elem_input_pt,&pt);

			if(!result)
				return false;

			this->_points->push_back(new cv::Point(pt));

			elem_input_pt = elem_input_pt->NextSiblingElement(_XML_FLOOR_ELEM_INPUT_POINT);
		}
	}

	return true;
}