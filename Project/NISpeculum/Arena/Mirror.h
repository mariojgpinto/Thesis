/**
 * @file Mirror.h
 * @author Mario Pinto (mariojgpinto@gmail.com)
 * @date May, 2013
 *
 * @brief
 *
 * @details
 *
 *
 */

#ifndef _MIRROR
#define _MIRROR

#include <opencv2/opencv.hpp>
#include <ToolBox.h>
//#include <ntk/geometry/plane.h>
#include <ntk/mesh/mesh_generator.h>

#include <ToolBoxCV.h>
#include <ToolBoxXML.h>

class Mirror{
	public:
		enum FLAGS{
			PLANE,
			MASK,
			INPUT
		};
		static const int _n_flags = 3;

	public:
		Mirror();
		~Mirror();

		//Setup
		void setup_variables();
		bool is_valid();

		//Flag
		void enable_flag(Mirror::FLAGS flag, bool value);
		bool check_flag(Mirror::FLAGS flag);

		//Construction
		void set_area(std::vector<cv::Point*>* points);
		void set_plane(double a, double b, double c, double d);
		void update_vertex(int index, cv::Point* point);
		
		//Processing
		void add_perspective_to_mesh(ntk::Mesh *mesh, ntk::RGBDImage* image);

		//Access
		cv::Mat1b* get_area_mask();
		ToolBox::Plane* get_plane();

		int get_n_vertexes();
		std::vector<cv::Point*>* get_vertexes();
		cv::Point* get_vertex(int index);

		//Storage
		bool save_to_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *elem, std::string* path, int index);
		bool load_from_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *root);

	private:
		void area_max_min(cv::Point* pt);

	private:
		//Flags
		bool _flags[Mirror::_n_flags];

		//Mirror Mask
		cv::Mat1b _area_mask;

		//Mirror Plane
		ToolBox::Plane _plane;

		//Area auxiliar variables
		int _area_max_width;
		int _area_max_height;
		int _area_min_width;
		int _area_min_height;

		//Mirror Points
		std::vector<cv::Point*>* _points;
		//std::vector<cv::Point*>* _user_points;
};


#endif