/**
 * @file Mirror.h
 * @author Mario Pinto (mariojgpinto@gmail.com)
 * @date July, 2013
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
#include <XnCppWrapper.h>

#include <ToolBox.h>
#include <ToolBoxCV.h>
#include <ToolBoxXML.h>

class Mirror{
	public:
		enum FLAGS{
			PLANE,
			AREA,
			INPUT
		};
		static const int _n_flags = 3;

	public:
		Mirror();
		~Mirror();

		//Setup
		void setup_variables();

		//Construction
		void set_area(std::vector<cv::Point*>* points);
		void set_plane(double a, double b, double c, double d);
		void update_vertex(int index, cv::Point* point);
		
		//Processing
		//void add_perspective_to_mesh(ntk::Mesh *mesh, ntk::RGBDImage* image);

		//Access
		int get_n_vertexes();
		std::vector<cv::Point*>* get_vertexes();
		cv::Point* get_vertex(int index);

		//Storage
		bool save_to_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *elem, std::string* path, int index);
		bool load_from_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *root);

	private:
		void area_max_min(cv::Point* pt);
	
	public:
		bool _ready[_n_flags];

		//Plane
		ToolBox::Plane _plane;

		//Masks
		cv::Mat _area_mask;

		int _n_points;
		XnPoint3D* _points;
		XnPoint3D* _points_mirrored;

		//Area auxiliar variables
		int _area_max_width;
		int _area_max_height;
		int _area_min_width;
		int _area_min_height;

	private:
		//Flags



		//Mirror Points
		std::vector<cv::Point*>* _user_points;
		//std::vector<cv::Point*>* _user_points;
};


#endif