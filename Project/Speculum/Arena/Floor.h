/**
 * @file Floor.h
 * @author Mario Pinto (mariojgpinto@gmail.com)
 * @date February, 2013
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
#include <ntk/camera/rgbd_image.h>
#include <ntk/geometry/plane.h>
#include <ntk/mesh/mesh_generator.h>

#include <ToolBoxCV.h>
#include <ToolBoxXML.h>

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
		void set_threshold(double thresh);
		bool is_valid();

		//Flag
		void enable_flag(Floor::FLAGS flag, bool value);
		bool check_flag(Floor::FLAGS flag);

		//Construction
		void set_area(std::vector<cv::Point*>* points);
		void set_plane(ntk::Plane *plane);
		void set_plane(double a, double b, double c, double d);
		void update_vertex(int index, cv::Point* point);
		void construct_floor_mask(ntk::RGBDImage* image);

		//Processing
		void add_perspective_to_mesh(ntk::Mesh* mesh, ntk::RGBDImage* image);

		//Access
		cv::Mat1b* get_area_mask();
		cv::Mat1b* get_floor_mask();
		double get_threshold();
		ntk::Plane* get_plane();

		int get_n_vertexes();
		std::vector<cv::Point*>* get_vertexes();
		cv::Point* get_vertex(int index);

		//Storage
		bool save_to_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *elem, std::string* path);
		bool load_from_file(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *root);

	private:
		void area_max_min(cv::Point* pt);

	private:
		bool _flags[Floor::_n_flags];

		//Floor Mask
		cv::Mat1b _floor_mask;
		
		//Floor Threshold
		double _thresh;

		//Area Mask
		cv::Mat1b _area_mask;

		//Area auxiliar variables
		int _area_max_width;
		int _area_max_height;
		int _area_min_width;
		int _area_min_height;

		//Floor Plane
		ntk::Plane _plane;

		//Floor Points
		std::vector<cv::Point*>* _points;

};

#endif