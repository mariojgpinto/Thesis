/**
 * @file Floor.h
 * @author Mario Pinto (mariojgpinto@gmail.com)
 * @date July, 2013
 *
 * @brief
 *
 * @details
 *
 *
 */

#ifndef _FLOOR
#define _FLOOR 

#include <opencv2/opencv.hpp>
#include <XnCppWrapper.h>

#include <ToolBox.h>
#include <ToolBoxCV.h>

namespace tinyxml2{
	class XMLDocument;
	class XMLElement;
}

class Floor{
	public:
		enum FLAGS{
			PLANE,
			AREA,
			FLOOR_MASK,
			INPUT
		};
		static const int _n_flags = 4;

	public:
		Floor();
		~Floor();

		//Setup
		void setup_variables();

		//Construction
		void set_area(std::vector<cv::Point*>* points);
		void set_plane(ToolBox::Plane *plane);
		void set_plane(double a, double b, double c, double d);
		void update_vertex(int index, cv::Point* point);
		//void construct_floor_mask(ntk::RGBDImage* image);

		//Processing
		//void add_perspective_to_mesh(ntk::Mesh* mesh, ntk::RGBDImage* image);

		//Access

		int get_n_vertexes();
		std::vector<cv::Point*>* get_vertexes();
		cv::Point* get_vertex(int index);

		//Storage
		bool save_to_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *elem, std::string* path);
		bool load_from_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *root);

	private:
		void area_max_min(cv::Point* pt);

	public:
		bool _ready[_n_flags];

		ToolBox::Plane _plane;

		//Masks
		cv::Mat _mask;
		cv::Mat _area_mask;

		//Floor Threshold
		double _thresh;

		int _n_points;
		XnPoint3D* _points;

	private:
		//Area auxiliar variables
		int _area_max_width;
		int _area_max_height;
		int _area_min_width;
		int _area_min_height;

		//Floor Plane
		

		//Floor Points
		std::vector<cv::Point*>* _user_points;

};

#endif