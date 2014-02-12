#include "Controller.h"

#include "PropertyManager.h"

#include "Floor.h"
#include "Mirror.h"
#include "ArenaXML.h"

//-----------------------------------------------------------------------------
// ADD MIRROR AND FLOOR
//-----------------------------------------------------------------------------
void Controller::add_mirror(Mirror* mirror){
	if(mirror){
		this->_mirrors.push_back(mirror);
		this->_n_mirrors++;
		this->construct_global_mask();
	}
}

void Controller::add_floor(Floor *floor){
	if(floor){
		this->_floor = floor;

		this->construct_global_mask();
	}
}

//-----------------------------------------------------------------------------
// MASK CONSTRUCTION
//-----------------------------------------------------------------------------
void Controller::construct_mirrors_masks(){
	cv::Mat temp;

	if(this->_n_mirrors){
		temp = cv::Mat::zeros(480,640,CV_8UC1);
		for(int i = 0 ; i < this->_mirrors.size() ; i++){
			if(this->_mirrors[i]->_ready[Mirror::AREA])
				cv::bitwise_or(temp,this->_mirrors[i]->_area_mask,temp);
		}
		this->_property_manager->_flag_processed[PropertyManager::P_MASK] = true;
	} else{
		temp = cv::Mat(480,640,CV_8UC1,(const uchar)255);
	}

	temp.copyTo(_mask_mirrors);
}

void Controller::construct_floor_mask(){
	this->_floor->_area_mask.copyTo(this->_mask_floor);
}

void Controller::construct_global_mask(){
	this->construct_mirrors_masks();
	this->construct_floor_mask();

	if(this->_floor->_ready[Floor::AREA]){
		cv::bitwise_or(this->_mask_mirrors,this->_mask_floor,this->_mask_mirrors_and_foor);
	}
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------

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
bool Controller::save_to_file(char* path, char* filename){
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

	//Storage Floor
	if(this->_floor){
		tinyxml2::XMLElement *elem_floor = doc->NewElement(_XML_ARENA_ELEM_FLOOR);

		this->_floor->save_to_file(doc,elem_floor,&xml_path);

		doc->LinkEndChild(elem_floor);
	}

	//Storage Mirrors
	if(this->_mirrors.size()){
		tinyxml2::XMLElement *elem_mirrors = doc->NewElement(_XML_ARENA_ELEM_MIRRORS);
		elem_mirrors->SetAttribute(_XML_ARENA_ATT_MIRRORS_N,this->_mirrors.size());

		for(unsigned int i = 0 ; i < this->_mirrors.size() ; i++){
			tinyxml2::XMLElement *elem_mirror = doc->NewElement(_XML_ARENA_ELEM_MIRROR);
			this->_mirrors.at(i)->save_to_file(doc,elem_mirror,&xml_path,i);
			elem_mirrors->LinkEndChild(elem_mirror);
		}

		doc->LinkEndChild(elem_mirrors);
	}

	//Save XML
	doc->SaveFile( xml_name.data( ) );

	return true;
}

bool Controller::load_from_file(char* file_path){
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
		//READ FLOOR
		if(strcmp(root->Value(),_XML_ARENA_ELEM_FLOOR) == 0){
			Floor* floor = new Floor();
			result = floor->load_from_file(&doc2,root);

			if(result){
				this->_floor = floor;
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