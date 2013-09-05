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
	if(mirror){
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
		this->_floor = floor;
		this->_flags[Arena::FLOOR] = true;
	}
}


//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
/*
ntk::Mesh* Arena::calculate_mesh(ntk::RGBDImage* image){
	//If no floor or no mirrors, return NULL
	if(!image) // || !this->_flags[Arena::FLOOR] || !this->_flags[Arena::MIRRORS])
		return NULL;

	bool floor = false;

	if(this->_floor->is_valid())
		floor = true;

	std::vector<Mirror*>* valid_mirrors = new std::vector<Mirror*>();

	for(unsigned int i = 0 ; i < this->_mirrors->size() ; i++){
		if(this->_mirrors->at(i)->is_valid())
			valid_mirrors->push_back(this->_mirrors->at(i));
	}

	//If none is ready, return NULL
	if(!floor && valid_mirrors->size() == 0)
		return NULL;

	ntk::Mesh* mesh = new ntk::Mesh();

	//Add Floor perspective to Mesh
	if(floor){
		this->_floor->add_perspective_to_mesh(mesh,image);
	}

	//Add each valid mirror perspective to Mesh
	for(unsigned int i = 0 ; i < valid_mirrors->size() ; i++){
		valid_mirrors->at(i)->add_perspective_to_mesh(mesh,image);
	}
	
	return mesh;
}
*/
//-----------------------------------------------------------------------------
// UTIL
//-----------------------------------------------------------------------------

//
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
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
/*
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
	fprintf(fp,"WIDTH %d\n", points->size() * (2*point_radius) * (2*point_radius));
	fprintf(fp,"HEIGHT 1\n");
	fprintf(fp,"VIEWPOINT 0 0 0 1 0 0 0\n");
	fprintf(fp,"POINTS %d\n", points->size() * (2*point_radius) * (2*point_radius));
	fprintf(fp,"DATA ascii\n");

	pcl::PointCloud<pcl::PointXYZ> cloud;
	 //Fill in the cloud data
	cloud.width  = points->size() *  (2*point_radius) * (2*point_radius);
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	cv::Point pt1;
	cv::Point3f pf1;
	int ac = 0;
	for(int i = -point_radius ; i < point_radius ; i++){
		for(int j = -point_radius ; j < point_radius ; j++){
			for(int k = 0 ; k < (int)points->size() ; k++){
				pt1.x = points->at(k)->x + i;
				pt1.y = points->at(k)->y + j;

				pf1 = image->calibration()->depth_pose->unprojectFromImage(pt1, image->depth()(pt1));

				if(pf1.x == 0 && pf1.y == 0 && pf1.z == 0){ 
					fprintf(fp,"nan nan nan\n");
				}
				else{
					fprintf(fp,"%f %f %f\n", pf1.x, pf1.y,pf1.z);
					cloud.points[ac].x = pf1.x;
					cloud.points[ac].y = pf1.y;
					cloud.points[ac].z = pf1.z;
					ac++;
				}
			}
		}
	}
	fclose(fp);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_PROSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud.makeShared ());
	seg.segment (*inliers, *coefficients);

	int s = coefficients->values.size();

	if(!s) return NULL;
	
	ntk::Plane* plane = new ntk::Plane(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
	
	return plane;
}
*/
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
 * @brief	
 * @details	
 *
 * @return 
 *
 */
int Arena::get_n_mirrors(){
	return this->_mirrors->size();
}

/**
 * @brief	
 * @details	
 *
 * @return 
 *
 */
cv::Mat* Arena::get_floor_mask(){
	if(this->_floor){
		return this->_floor->get_area_mask();
	}
	return NULL;
}

/**
 * @brief	
 * @details	
 *
 * @param[in]	idx
 *				
 *
 * @return 
 *
 */
cv::Mat* Arena::get_mirror_mask(int idx){
	if(this->_mirrors && idx >= 0 && idx < this->_mirrors->size()){
		return this->_mirrors->at(idx)->get_area_mask();
	}
	return NULL;
}

/**
 * @brief	
 * @details	
 *
 * @return 
 *
 */
cv::Mat* Arena::get_mirror_masks(){
	if(this->_mirrors){
		cv::Mat temp;
		this->_mirrors->at(0)->get_area_mask()->copyTo(temp);
		for(int i = 1 ; i < this->_mirrors->size() ; i++){
			cv::bitwise_or(temp,*this->_mirrors->at(0)->get_area_mask(),temp);
		}

		return &temp;
	}
	return NULL;
}

/**
 * @brief	
 * @details	
 *
 * @return 
 *
 */
cv::Mat* Arena::get_all_mask(){
	if(this->_mirrors || this->_floor){
		cv::Mat temp;

		if(this->_floor){
			this->_floor->get_area_mask()->copyTo(temp);
		}
		if(this->_mirrors){
			if(this->_floor){
				for(int i = 0 ; i < this->_mirrors->size() ; i++){
					cv::bitwise_or(temp,*this->_mirrors->at(0)->get_area_mask(),temp);
				}
			}
			else{
				this->_mirrors->at(0)->get_area_mask()->copyTo(temp);
				for(int i = 1 ; i < this->_mirrors->size() ; i++){
					cv::bitwise_or(temp,*this->_mirrors->at(0)->get_area_mask(),temp);
				}
			}
		}

		return &temp;
	}
	return NULL;
}


//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------

#include "ArenaXML.h"

/**
 * @brief	Storages the current Arena to file.
 * @details	.
 *
 * @param	filename
 *			.
 *
 * @return
 *
 */
bool Arena::save_to_file(char* path, char* filename){
	std::string xml_name;
	std::string xml_path;

	if(path){
		//TODO check if ends with Slash
		xml_path.assign(path);
		//xml_path.append("\\");
	}
	else{
		//xml_path.assign(".\\");
	}


	xml_name.assign(xml_path);
	xml_name.append("\\");
	if(filename){
		xml_name.append(filename);
		//TODO check if ends with .xml
		xml_name.append(".xml");
	}
	else{
		xml_name.append("arena.xml");
	}
	
	tinyxml2::XMLDocument *doc =  ToolBoxXML::create_xml_doc();

	//Storage Flags
	{
		tinyxml2::XMLElement *elem_flags = doc->NewElement(_XML_ARENA_ELEM_FLAGS);

			//Floor Flag
			tinyxml2::XMLElement *elem_flag_floor = doc->NewElement(_XML_ARENA_ELEM_FLAG_FLOOR);
			elem_flag_floor->SetAttribute(_XML_FLAG,this->_flags[Arena::FLOOR]);
			elem_flags->LinkEndChild(elem_flag_floor);

			//Mirrors Flag
			tinyxml2::XMLElement *elem_flag_mirrors = doc->NewElement(_XML_ARENA_ELEM_FLAG_MIRRORS);
			elem_flag_mirrors->SetAttribute(_XML_FLAG,this->_flags[Arena::MIRRORS]);
			elem_flags->LinkEndChild(elem_flag_mirrors);

		doc->LinkEndChild(elem_flags);
	}
	//Storage Floor
	if(this->_flags[Arena::FLOOR] && this->_floor){
		tinyxml2::XMLElement *elem_floor = doc->NewElement(_XML_ARENA_ELEM_FLOOR);

		this->_floor->save_to_file(doc,elem_floor,&xml_path);

		doc->LinkEndChild(elem_floor);
	}

	//Storage Mirrors
	if(this->_flags[Arena::MIRRORS] && this->_mirrors){
		tinyxml2::XMLElement *elem_mirrors = doc->NewElement(_XML_ARENA_ELEM_MIRRORS);
		elem_mirrors->SetAttribute(_XML_ARENA_ATT_MIRRORS_N,this->_mirrors->size());

		for(unsigned int i = 0 ; i < this->_mirrors->size() ; i++){
			tinyxml2::XMLElement *elem_mirror = doc->NewElement(_XML_ARENA_ELEM_MIRROR);
			this->_mirrors->at(i)->save_to_file(doc,elem_mirror,&xml_path,i);
			elem_mirrors->LinkEndChild(elem_mirror);
		}

		doc->LinkEndChild(elem_mirrors);
	}


	//Save XML
	doc->SaveFile( xml_name.data( ) );

	return true;
}

bool Arena::load_from_file(char* file_path){
	if(!file_path) return false;

	//XML Variables
	tinyxml2::XMLDocument doc2;
	tinyxml2::XMLError error;
	
	//Open XML
	error = doc2.LoadFile( file_path );
	if(error) return NULL;

	tinyxml2::XMLElement *root = doc2.RootElement();

	bool result = false;

	while(root){
		//READ FLAGS
		if(strcmp(root->Value(),_XML_ARENA_ELEM_FLAGS) == 0){
			tinyxml2::XMLElement *elem_flag_floor = root->FirstChildElement(_XML_ARENA_ELEM_FLAG_FLOOR);

			if(elem_flag_floor){
				bool temp_flag = false;
				error = elem_flag_floor->QueryBoolAttribute(_XML_FLAG,&temp_flag);
				if(error) this->enable_flag(Arena::FLOOR,false);
				else this->enable_flag(Arena::FLOOR,temp_flag);
			} else this->enable_flag(Arena::FLOOR,false);

			tinyxml2::XMLElement *elem_flag_mirrors = root->FirstChildElement(_XML_ARENA_ELEM_FLAG_MIRRORS);

			if(elem_flag_mirrors){
				bool temp_flag = false;
				error = elem_flag_mirrors->QueryBoolAttribute(_XML_FLAG,&temp_flag);
				if(error) this->enable_flag(Arena::MIRRORS,false);
				else this->enable_flag(Arena::MIRRORS,temp_flag);
			} else this->enable_flag(Arena::MIRRORS,false);
		} else
		//READ FLOOR
		if(strcmp(root->Value(),_XML_ARENA_ELEM_FLOOR) == 0){
			Floor* floor = new Floor();

			result = floor->load_from_file(&doc2,root);

			if(result){
				this->add_floor(floor);
			}
			else{
				return false;
			}
		} else
		//READ MIRRORS
		if(strcmp(root->Value(),_XML_ARENA_ELEM_FLAG_MIRRORS) == 0){
			int n_mirrors = 0;

			error = root->QueryIntAttribute(_XML_ARENA_ATT_MIRRORS_N,&n_mirrors);
			
			if(error) return false;
			
			tinyxml2::XMLElement *elem_mirror = root->FirstChildElement(_XML_ARENA_ELEM_MIRROR);
			
			for(int i = 0 ; i < n_mirrors ; i++){
				if(!elem_mirror) return false;

				Mirror* mirror = new Mirror();


				result = mirror->load_from_file(&doc2,elem_mirror);

				if(result){
					this->add_mirror(mirror);
				}
				else{
					return false;
				}

				elem_mirror = elem_mirror->NextSiblingElement(_XML_ARENA_ELEM_MIRROR);
			}
		}

		root = root->NextSiblingElement();
	}

	return true;
}