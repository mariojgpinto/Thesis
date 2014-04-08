/**
 * @file Floor.cpp
 * @author Mario Pinto (mariojgpinto@gmail.com)
 * @date February, 2013
 *
 * @brief
 *
 * @details
 *
 *
 */

#include "Floor.h"

#include <ToolBoxXML.h>

//-----------------------------------------------------------------------------
// CONTRUCTORS
//-----------------------------------------------------------------------------
Floor::Floor():
	//_area_mask(480,640,(const uchar)255),
	_mask(480,640,(const uchar)255),
	_depth_min(500),
	_depth_max(3000)
	{
	this->setup_variables();
}

Floor::~Floor(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/*
 *
 */
void Floor::setup_variables(){
	for(int i = 0 ; i < _n_flags ; i++)
		this->_ready[i] = false;

	this->_user_points = new std::vector<cv::Point*>();

	this->_area_max_width = INT_MIN;
	this->_area_max_height = INT_MIN;
	this->_area_min_width = INT_MAX;
	this->_area_min_height = INT_MAX;

	this->_thresh = 10;

	this->_plane.set(0,0,0,0);

	this->_color = false;
	this->_color_r = 255;
	this->_color_b = 255;
	this->_color_g = 255;
}

//-----------------------------------------------------------------------------
// FLOOR CONSTRUCTION
//-----------------------------------------------------------------------------
/**
 * @brief Sets the area of the Floor and constructs the Area Mask.
 * @details 
 *
 * @pre The input vector needs to have at least three points.
 *
 * @warning The function <c> cv::fillPoly <\c> may generate an exception. 
 *			This exception is caught and the mask is @b not created neither the is flag is set to true.
 *
 * @param points Vector with the points selected to create the area mask.
 *
 */
void Floor::set_area(std::vector<cv::Point*>* points){
	if(!points || points->size() < 3) return;

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
		this->_user_points->clear();
		for(unsigned int i = 0 ; i < points->size() ; i++){
			if(points->at(i)){
				this->_user_points->push_back(new cv::Point(*points->at(i)));
			}
		}

		mask.assignTo(this->_area_mask);
		this->_ready[Floor::AREA] = true;
		this->_ready[Floor::INPUT] = true;
	}
}

/**
 * @brief
 * @details
 *
 * @param pt
 *
 */
void Floor::area_max_min(cv::Point* pt){
	if(pt->x < this->_area_min_width)
		this->_area_min_width = pt->x;

	if(pt->y < this->_area_min_height)
		this->_area_min_height = pt->y;

	if(pt->x > this->_area_max_width)
		this->_area_max_width = pt->x;
	
	if(pt->y > this->_area_max_height)
		this->_area_max_height = pt->y;
}

/**
 *
 * @param plane
 *
 */
void Floor::set_plane(ToolBox::Plane *plane){
	if(!plane) return;

	this->_plane.set(plane->_a,plane->_b,plane->_c,plane->_d);
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
void Floor::set_plane(double a, double b, double c, double d){
	this->_plane.set(a,b,c,d);
	//this->_plane.a = a;
	//this->_plane.b = b;
	//this->_plane.c = c;
	//this->_plane.d = d;
	
	this->_ready[Floor::PLANE] = true;
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
void Floor::update_vertex(int index, cv::Point* point){
	if(point && index >= 0 && index < (int)this->_user_points->size()){
		this->_user_points->at(index)->x = point->x;
		this->_user_points->at(index)->y = point->y;
		
		std::vector<cv::Point*> aux;

		for(unsigned int i = 0 ; i < this->_user_points->size() ; i++){
			aux.push_back(new cv::Point(*this->_user_points->at(i)));
		}

		this->set_area(&aux);
	}
}

#include <opencv2\core\mat.hpp>

bool is_yx_in_range(const cv::Mat& image, int y, int x)
{ return (x >= 0) && (y >= 0) && (x < image.cols) && (y < image.rows); }

float estimateErrorFromDepth(float depth, float max_depth_at_1m)
{
    if (depth < 1) return max_depth_at_1m;
    return depth * max_depth_at_1m;
}
//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
/*
void Floor::add_perspective_to_mesh(ntk::Mesh* mesh, ntk::RGBDImage* image){
	if(!mesh || !image) return;

	//image max and min to 
	int max_width;
	int max_height;
	int min_width;
	int min_height;

	if(this->_ready[Floor::AREA]){
		max_width = this->_area_max_width;
		max_height = this->_area_max_height;
		min_width = this->_area_min_width;
		min_height = this->_area_min_height;
	}
	else{
		max_width = image->depth().cols;
		max_height = image->depth().rows;
		min_width = 0;
		min_height = 0;
	}

	cv::Mat1f aux_depth = image->depth();
	cv::Mat1b aux_depth_mask = image->depthMask();
	cv::Mat3b aux_color = image->rgb();
	ntk::Pose3D* aux_pose = image->calibration()->depth_pose;
	ntk::Pose3D* aux_pose_rgb = image->calibration()->rgb_pose;

	//Create a mask with all existing masks
	cv::Mat1b mega_mask;
	image->depthMask().copyTo(mega_mask);

	if(this->_ready[Floor::AREA]){
		cv::bitwise_and(mega_mask,this->_area_mask,mega_mask);
	}
	if(this->_ready[Floor::FLOOR_MASK]){
		cv::bitwise_and(mega_mask,this->_floor_mask,mega_mask);
	}


	mesh->clear();
	image->rgb().copyTo(mesh->texture);   
	mesh->vertices.reserve(aux_depth.cols*aux_depth.rows);
    mesh->texcoords.reserve(aux_depth.cols*aux_depth.rows);
    mesh->colors.reserve(aux_depth.cols*aux_depth.rows);
	cv::Mat1i vertice_map(aux_depth.size());
	vertice_map = -1;

	for(int i = min_width ; i < max_width ; i++){
		for(int j = min_height ; j < max_height ; j++){
			cv::Point p(i,j);
			//If the point is part of the mask
			if(mega_mask(p)){
				//Unprojected point
				cv::Point3f pf = aux_pose->unprojectFromImage(p, aux_depth(p));
				
				//Color vector to pass from RGB to BGR
				cv::Vec3b color = aux_color(p);
				
				//Create the Surfel, add porperties and add to mesh
				ntk::Surfel surf;
				surf.location = cv::Point3f(pf);
				surf.color = cv::Vec3f(color.val[2],color.val[1],color.val[0]);
				mesh->addPointFromSurfel(surf);
				
				//cv::Point3f p2d_rgb = aux_pose_rgb->projectToImage(pf);
				//cv::Point2f texcoords = cv::Point2f(p2d_rgb.x/aux_color.cols, p2d_rgb.y/aux_color.rows);
				//vertice_map(j,i) = mesh->vertices.size();
				//mesh->vertices.push_back(pf);
				//mesh->colors.push_back(cv::Vec3f(color.val[2],color.val[1],color.val[0]));
				//mesh->texcoords.push_back(texcoords);
			}
		}
	}

	//double m_max_delta_depth = 0.05f;

	//for(int i = 0 ; i < vertice_map.cols ; i++){
	//	for(int j = min_height ; j < vertice_map.rows ; j++){
	//		 if (vertice_map(j,i) < 0)
	//			continue;

	//		if ((i < vertice_map.cols - 1) &&  (j < vertice_map.rows - 1) &&
	//			(vertice_map(j+1,i)>=0) && (vertice_map(j,i+1) >= 0) &&
	//			(std::abs(aux_depth(j,i) - aux_depth(j+1, i)) < m_max_delta_depth) &&
	//			(std::abs(aux_depth(j,i) - aux_depth(j, i+1)) < m_max_delta_depth))
	//		{
	//			ntk::Face f;
	//			f.indices[2] = vertice_map(j,i);
	//			f.indices[1] = vertice_map(j,i+1);
	//			f.indices[0] = vertice_map(j+1,i);
	//			mesh->faces.push_back(f);
	//		}

	//		float delta_depth = estimateErrorFromDepth(aux_depth(j,i), m_max_delta_depth);

	//		if ((i > 0) &&  (j < vertice_map.rows - 1) &&
	//			(vertice_map(j+1,i)>=0) && (vertice_map(j+1,i-1) >= 0) &&
	//			(std::abs(aux_depth(j,i) - aux_depth(j+1, i)) < delta_depth) &&
	//			(std::abs(aux_depth(j,i) - aux_depth(j+1, i-1)) < delta_depth))
	//		{
	//			ntk::Face f;
	//			f.indices[2] = vertice_map(j,i);
	//			f.indices[1] = vertice_map(j+1,i);
	//			f.indices[0] = vertice_map(j+1,i-1);
	//			mesh->faces.push_back(f);
	//		}
	//	}
	//}

	// mesh->computeNormalsFromFaces();
}
*/
/**
 *
 */
//void Floor::construct_floor_mask(ntk::RGBDImage* image){
//	if(!image || !this->_ready[Floor::PLANE]) return;
//
//	cv::Size size = image->depth().size();
//
//	//Create and initialize the mask
//	this->_floor_mask.create(size);
//	this->_floor_mask.zeros(size);
//
//	//Create Auxiliar Map with Depth
//	cv::Mat1f depth;
//
//	if(this->_ready[Floor::AREA])
//		image->depth().copyTo(depth,this->_area_mask);
//	else
//		image->depth().copyTo(depth);
//
//	ntk::Pose3D* pose = image->calibration()->depth_pose;
//	double dist = 0.0;
//
//	for(int j = 0 ; j < depth.rows ; j++){
//		for(int i = 0 ; i < depth.cols ; i++){
//			cv::Point p(i,j);
//			if(depth(p)){
//				cv::Point3f pf = pose->unprojectFromImage(p, depth(p));
//
//				dist = this->_plane.distance_to_plane(pf.x,pf.y,pf.z);
//
//				if(dist < this->_thresh){
//					this->_floor_mask[j][i] = 0;
//				}
//				else{
//					this->_floor_mask[j][i] = 255;
//				}
//			}
//			else{
//				this->_floor_mask[j][i] = 0;
//			}
//		}
//	}
//
//	this->_ready[Floor::FLOOR_MASK] = true;
//}


//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// ACCESS - FLOOR MANAGEMENT
//-----------------------------------------------------------------------------
/** 
 *
 *
 * @return
 *
 *
 */
int Floor::get_n_vertexes(){
	return this->_user_points->size();
}

/** 
 *
 *
 * @return
 *
 *
 */
std::vector<cv::Point*>* Floor::get_vertexes(){
	return this->_user_points;
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
cv::Point* Floor::get_vertex(int index){
	if(index >= 0 && index < (int)this->_user_points->size()){
		return this->_user_points->at(index);
	}
	return NULL;
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
#include "ArenaXML.h"

bool Floor::save_to_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *elem, std::string* path){
	if(!doc || !elem || !path) return false;

	bool result = false;

	//FLAGS
	{
		tinyxml2::XMLElement *elem_flags = doc->NewElement(_XML_FLOOR_ELEM_FLAGS);

			//PLANE FLAG
			tinyxml2::XMLElement *elem_flag_plane = doc->NewElement(_XML_FLOOR_ELEM_FLAG_PLANE);
			elem_flag_plane->SetAttribute(_XML_FLAG,this->_ready[Floor::PLANE]);
			elem_flags->LinkEndChild(elem_flag_plane);

			//AREA FLAG
			tinyxml2::XMLElement *elem_flag_area = doc->NewElement(_XML_FLOOR_ELEM_FLAG_AREA);
			elem_flag_area->SetAttribute(_XML_FLAG,this->_ready[Floor::AREA]);
			elem_flags->LinkEndChild(elem_flag_area);

			//MASK FLAG
			tinyxml2::XMLElement *elem_flag_mask = doc->NewElement(_XML_FLOOR_ELEM_FLAG_FLOOR_MASK);
			elem_flag_mask->SetAttribute(_XML_FLAG,this->_ready[Floor::FLOOR_MASK]);
			elem_flags->LinkEndChild(elem_flag_mask);

			//INPUT FLAG
			tinyxml2::XMLElement *elem_flag_input = doc->NewElement(_XML_FLOOR_ELEM_FLAG_INPUT);
			elem_flag_input->SetAttribute(_XML_FLAG,this->_ready[Floor::INPUT]);
			elem_flags->LinkEndChild(elem_flag_input);

		elem->LinkEndChild(elem_flags);
	}

	//PLANE
	if(this->_ready[Floor::PLANE])
	{
		tinyxml2::XMLElement *elem_plane = doc->NewElement(_XML_FLOOR_ELEM_PLANE);
			
			tinyxml2::XMLElement *elem_plane_a = doc->NewElement(_XML_FLOOR_ELEM_PLANE_A);
			elem_plane_a->SetAttribute(_XML_VALUE,this->_plane._a);
			elem_plane->LinkEndChild(elem_plane_a);

			tinyxml2::XMLElement *elem_plane_b = doc->NewElement(_XML_FLOOR_ELEM_PLANE_B);
			elem_plane_b->SetAttribute(_XML_VALUE,this->_plane._b);
			elem_plane->LinkEndChild(elem_plane_b);

			tinyxml2::XMLElement *elem_plane_c = doc->NewElement(_XML_FLOOR_ELEM_PLANE_C);
			elem_plane_c->SetAttribute(_XML_VALUE,this->_plane._c);
			elem_plane->LinkEndChild(elem_plane_c);

			tinyxml2::XMLElement *elem_plane_d = doc->NewElement(_XML_FLOOR_ELEM_PLANE_D);
			elem_plane_d->SetAttribute(_XML_VALUE,this->_plane._d);
			elem_plane->LinkEndChild(elem_plane_d);

		elem->LinkEndChild(elem_plane);
	}

	//AREA MASK
	if(this->_ready[Floor::AREA])
	{
		tinyxml2::XMLElement *elem_area = doc->NewElement(_XML_FLOOR_ELEM_AREA);
			
			tinyxml2::XMLElement *elem_area_mask = doc->NewElement(_XML_FLOOR_ELEM_AREA_MASK);
			
			result = ToolBoxXML::cv_save_image(elem_area,elem_area_mask,(char*)path->data(),_XML_FLOOR_FILE_AREA_MASK,this->_area_mask);
			if(!result) return false;

			tinyxml2::XMLElement *elem_area_limits = doc->NewElement(_XML_FLOOR_ELEM_AREA_LIMITS);
			
				tinyxml2::XMLElement *elem_area_limits_max_w = doc->NewElement(_XML_FLOOR_ELEM_AREA_MAX_WIDTH);
				elem_area_limits_max_w->SetAttribute(_XML_VALUE,this->_area_max_width);
				elem_area_limits->LinkEndChild(elem_area_limits_max_w);

				tinyxml2::XMLElement *elem_area_limits_min_w = doc->NewElement(_XML_FLOOR_ELEM_AREA_MIN_WIDTH);
				elem_area_limits_min_w->SetAttribute(_XML_VALUE,this->_area_min_width);
				elem_area_limits->LinkEndChild(elem_area_limits_min_w);

				tinyxml2::XMLElement *elem_area_limits_max_h = doc->NewElement(_XML_FLOOR_ELEM_AREA_MAX_HEIGHT);
				elem_area_limits_max_h->SetAttribute(_XML_VALUE,this->_area_max_height);
				elem_area_limits->LinkEndChild(elem_area_limits_max_h);

				tinyxml2::XMLElement *elem_area_limits_min_h = doc->NewElement(_XML_FLOOR_ELEM_AREA_MIN_HEIGHT);
				elem_area_limits_min_h->SetAttribute(_XML_VALUE,this->_area_min_height);
				elem_area_limits->LinkEndChild(elem_area_limits_min_h);

			elem_area->LinkEndChild(elem_area_limits);

		elem->LinkEndChild(elem_area);
	}

	//FLOOR MASK
	if(this->_ready[Floor::FLOOR_MASK])
	{
		tinyxml2::XMLElement *elem_floor_mask = doc->NewElement(_XML_FLOOR_ELEM_MASK);
			
			tinyxml2::XMLElement *elem_floor_mask_mask = doc->NewElement(_XML_FLOOR_ELEM_MASK_MASK);

			result = ToolBoxXML::cv_save_image(elem_floor_mask,elem_floor_mask_mask,(char*)path->data(),_XML_FLOOR_FILE_FLOOR_MASK,this->_mask);
			if(!result) return false;

			tinyxml2::XMLElement *elem_floor_mask_thresh = doc->NewElement(_XML_FLOOR_ELEM_MASK_THRESH);
			elem_floor_mask_thresh->SetAttribute(_XML_VALUE,this->_thresh);
			elem_floor_mask->LinkEndChild(elem_floor_mask_thresh);

		elem->LinkEndChild(elem_floor_mask);
	}

	//INPUT
	if(this->_ready[Floor::INPUT])
	{
		tinyxml2::XMLElement *elem_input = doc->NewElement(_XML_FLOOR_ELEM_INPUT);
			
			tinyxml2::XMLElement *elem_input_n = doc->NewElement(_XML_FLOOR_ELEM_INPUT_N);
			elem_input_n->SetAttribute(_XML_VALUE,this->_user_points->size());
			elem_input->LinkEndChild(elem_input_n);
			
			tinyxml2::XMLElement *elem_input_vertexes = doc->NewElement(_XML_FLOOR_ELEM_INPUT_POINTS);
			
			for(unsigned int i = 0 ; i < this->_user_points->size() ; i++){
				tinyxml2::XMLElement *elem_input_vertex = doc->NewElement(_XML_FLOOR_ELEM_INPUT_POINT);
				result = ToolBoxXML::cv_add_point(elem_input_vertex,this->_user_points->at(i));
				
				if(!result) return false;
				
				elem_input_vertexes->LinkEndChild(elem_input_vertex);
			}

			elem_input->LinkEndChild(elem_input_vertexes);


		elem->LinkEndChild(elem_input);
	}

	//DEPTH
	{
		tinyxml2::XMLElement *elem_depth = doc->NewElement(_XML_FLOOR_ELEM_DEPTH);
			
			tinyxml2::XMLElement *elem_depth_min = doc->NewElement(_XML_FLOOR_ELEM_DEPTH_MIN);
			elem_depth_min->SetAttribute(_XML_VALUE,this->_depth_min);
			elem_depth->LinkEndChild(elem_depth_min);

			tinyxml2::XMLElement *elem_depth_max = doc->NewElement(_XML_FLOOR_ELEM_DEPTH_MAX);
			elem_depth_max->SetAttribute(_XML_VALUE,this->_depth_max);
			elem_depth->LinkEndChild(elem_depth_max);

			tinyxml2::XMLElement *elem_threshold = doc->NewElement(_XML_FLOOR_ELEM_THRESHOLD);
			elem_threshold->SetAttribute(_XML_VALUE,this->_thresh);
			elem_depth->LinkEndChild(elem_threshold);
			

		elem->LinkEndChild(elem_depth);
	}

	return true;
}

bool Floor::load_from_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *root){
	if(!doc || !root) return false;

	tinyxml2::XMLError error;

	//READ FLAGS
	{
		tinyxml2::XMLElement *elem_flags = root->FirstChildElement(_XML_FLOOR_ELEM_FLAGS);

		if(!elem_flags) return false;

		//PLANE FLAG
		tinyxml2::XMLElement *elem_flag_plane = elem_flags->FirstChildElement(_XML_FLOOR_ELEM_FLAG_PLANE);

		if(elem_flag_plane){
			bool temp_flag = false;
			error = elem_flag_plane->QueryBoolAttribute(_XML_FLAG,&temp_flag);
			if(error) this->_ready[Floor::PLANE] = false;
			else this->_ready[Floor::PLANE] = temp_flag;
		} else this->_ready[Floor::PLANE] = false;

		//AREA FLAG
		tinyxml2::XMLElement *elem_flag_area = elem_flags->FirstChildElement(_XML_FLOOR_ELEM_FLAG_AREA);

		if(elem_flag_area){
			bool temp_flag = false;
			error = elem_flag_area->QueryBoolAttribute(_XML_FLAG,&temp_flag);
			if(error) this->_ready[Floor::AREA] = false;
			else this->_ready[Floor::AREA] = temp_flag;
		} else this->_ready[Floor::AREA] = false;

		//MASK FLAG
		tinyxml2::XMLElement *elem_flag_mask = elem_flags->FirstChildElement(_XML_FLOOR_ELEM_FLAG_FLOOR_MASK);

		if(elem_flag_mask){
			bool temp_flag = false;
			error = elem_flag_mask->QueryBoolAttribute(_XML_FLAG,&temp_flag);
			if(error) this->_ready[Floor::FLOOR_MASK] = false;
			else this->_ready[Floor::FLOOR_MASK] = temp_flag;
		} else this->_ready[Floor::FLOOR_MASK] = false;

		//INPUT FLAG
		tinyxml2::XMLElement *elem_flag_input = elem_flags->FirstChildElement(_XML_FLOOR_ELEM_FLAG_INPUT);

		if(elem_flag_input){
			bool temp_flag = false;
			error = elem_flag_input->QueryBoolAttribute(_XML_FLAG,&temp_flag);
			if(error) this->_ready[Floor::INPUT] = false;
			else this->_ready[Floor::INPUT] = temp_flag;
		} else this->_ready[Floor::INPUT] = false;
	}
	printf("FLAGS");

	//READ PLANE
	if(this->_ready[Floor::PLANE]){
		tinyxml2::XMLElement *elem_plane = root->FirstChildElement(_XML_FLOOR_ELEM_PLANE);

		if(!elem_plane) return false;

		double a,b,c,d;

		//READ 'A' VALUE
		tinyxml2::XMLElement *elem_floor_plane_a = elem_plane->FirstChildElement(_XML_FLOOR_ELEM_PLANE_A);
		if(elem_floor_plane_a){
			error = elem_floor_plane_a->QueryDoubleAttribute(_XML_VALUE,&a);
			if(error)
				return false;
		}
		//READ 'B' VALUE
		tinyxml2::XMLElement *elem_floor_plane_b = elem_plane->FirstChildElement(_XML_FLOOR_ELEM_PLANE_B);
		if(elem_floor_plane_b){
			error = elem_floor_plane_b->QueryDoubleAttribute(_XML_VALUE,&b);
			if(error)
				return  false;
		}
		//READ 'C' VALUE
		tinyxml2::XMLElement *elem_floor_plane_c = elem_plane->FirstChildElement(_XML_FLOOR_ELEM_PLANE_C);
		if(elem_floor_plane_c){
			error = elem_floor_plane_c->QueryDoubleAttribute(_XML_VALUE,&c);
			if(error)
				return false;
		}
		//READ 'D' VALUE
		tinyxml2::XMLElement *elem_floor_plane_d = elem_plane->FirstChildElement(_XML_FLOOR_ELEM_PLANE_D);
		if(elem_floor_plane_d){
			error = elem_floor_plane_d->QueryDoubleAttribute(_XML_VALUE,&d);
			if(error)
				return  false;
		}

		if(this->_ready[Floor::PLANE]){
			this->_plane.set(a,b,c,d);
		}
	}
	printf("PLANE");
	//READ AREA 
	if(this->_ready[Floor::AREA]){
		tinyxml2::XMLElement *elem_area = root->FirstChildElement(_XML_FLOOR_ELEM_AREA);

		if(!elem_area) return false;
		printf("Image");
		bool result = ToolBoxXML::cv_load_image_xml(elem_area,_XML_FLOOR_ELEM_AREA_MASK,_area_mask,0);
		if(!result) 
			return false;
		
		tinyxml2::XMLElement *elem_area_limits = elem_area->FirstChildElement(_XML_FLOOR_ELEM_AREA_LIMITS);

		if(!elem_area_limits) return false;

		tinyxml2::XMLElement *elem_area_limits_max_w = elem_area_limits->FirstChildElement(_XML_FLOOR_ELEM_AREA_MAX_WIDTH);
		if(elem_area_limits_max_w){
			error = elem_area_limits_max_w->QueryIntAttribute(_XML_VALUE,&this->_area_max_width);
			if(error) return false;
		}else return false;

		tinyxml2::XMLElement *elem_area_limits_min_w = elem_area_limits->FirstChildElement(_XML_FLOOR_ELEM_AREA_MIN_WIDTH);
		if(elem_area_limits_min_w){
			error = elem_area_limits_min_w->QueryIntAttribute(_XML_VALUE,&this->_area_min_width);
			if(error) return false;
		}else return false;

		tinyxml2::XMLElement *elem_area_limits_max_h = elem_area_limits->FirstChildElement(_XML_FLOOR_ELEM_AREA_MAX_HEIGHT);
		if(elem_area_limits_max_h){
			error = elem_area_limits_max_h->QueryIntAttribute(_XML_VALUE,&this->_area_max_height);
			if(error) return false;
		}else return false;

		tinyxml2::XMLElement *elem_area_limits_min_h = elem_area_limits->FirstChildElement(_XML_FLOOR_ELEM_AREA_MIN_HEIGHT);
		if(elem_area_limits_min_h){
			error = elem_area_limits_min_h->QueryIntAttribute(_XML_VALUE,&this->_area_min_height);
			if(error) return false;
		}else return false;
	}
	printf("AREA");
	//READ MASK
	if(this->_ready[Floor::FLOOR_MASK]){
		tinyxml2::XMLElement *elem_mask = root->FirstChildElement(_XML_FLOOR_ELEM_MASK);

		if(!elem_mask) return false;

		bool result = ToolBoxXML::cv_load_image_xml(elem_mask,_XML_FLOOR_ELEM_MASK_MASK,(cv::Mat1b)this->_mask);
		if(!result) 
			return false;

		tinyxml2::XMLElement *elem_mask_tresh = elem_mask->FirstChildElement(_XML_FLOOR_ELEM_MASK_THRESH);

		if(!elem_mask_tresh) return false;

		error = elem_mask_tresh->QueryDoubleAttribute(_XML_VALUE,&this->_thresh);
		if(error) return false;
	}
	printf("MASK");
	//READ INPUT
	if(this->_ready[Floor::INPUT]){
		tinyxml2::XMLElement *elem_input = root->FirstChildElement(_XML_FLOOR_ELEM_INPUT);

		if(!elem_input) return false;

		int n_points = 0;
		tinyxml2::XMLElement *elem_input_n_pts = elem_input->FirstChildElement(_XML_FLOOR_ELEM_INPUT_N);
		if(!elem_input_n_pts) return false;

		error = elem_input_n_pts->QueryIntAttribute(_XML_VALUE,&n_points);
		if(error) return false;

		tinyxml2::XMLElement *elem_input_pts = elem_input->FirstChildElement(_XML_FLOOR_ELEM_INPUT_POINTS);
		if(!elem_input_pts) return false;

		tinyxml2::XMLElement *elem_input_pt = elem_input_pts->FirstChildElement(_XML_FLOOR_ELEM_INPUT_POINT);

		for(int i = 0 ; i < n_points ; i++){
			if(!elem_input_pt) return false;

			cv::Point pt;
			bool result = ToolBoxXML::cv_read_point(elem_input_pt,&pt);

			if(!result)
				return false;

			this->_user_points->push_back(new cv::Point(pt));

			elem_input_pt = elem_input_pt->NextSiblingElement(_XML_FLOOR_ELEM_INPUT_POINT);
		}
	}
	printf("INPUT");
	//DEPTH
	{
		tinyxml2::XMLElement *elem_depth = root->FirstChildElement(_XML_FLOOR_ELEM_DEPTH);

		if(!elem_depth) return false;

		int depth_min = 0;
		tinyxml2::XMLElement *elem_depth_min = elem_depth->FirstChildElement(_XML_FLOOR_ELEM_DEPTH_MIN);
		if(!elem_depth_min) return false;

		error = elem_depth_min->QueryIntAttribute(_XML_VALUE,&depth_min);
		if(error) return false;

		int depth_max = 0;
		tinyxml2::XMLElement *elem_depth_max = elem_depth->FirstChildElement(_XML_FLOOR_ELEM_DEPTH_MAX);
		if(!elem_depth_max) return false;

		error = elem_depth_max->QueryIntAttribute(_XML_VALUE,&depth_max);
		if(error) return false;

		int threshold = 0;
		tinyxml2::XMLElement *elem_threshold = elem_depth->FirstChildElement(_XML_FLOOR_ELEM_THRESHOLD);
		if(!elem_threshold) return false;

		error = elem_threshold->QueryIntAttribute(_XML_VALUE,&threshold);
		if(error) return false;

		this->_depth_min = depth_min;
		this->_depth_max = depth_max;
		this->_thresh = threshold;
	}
	printf("DEPTH");
	return true;
}