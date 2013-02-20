/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef _MIRROR
#define _MIRROR

#include <opencv2\opencv.hpp>
#include <ntk\geometry\plane.h>
#include <ToolBoxCV.h>

class Mirror{
	public:
		enum FLAGS{
			MASK,
			INPUT,
			PLANE
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
		void set_plane(int a, int b, int c, int d);
		void update_vertex(int index, cv::Point* point);

		//Access
		cv::Mat1b* get_mask();
		ntk::Plane* get_plane();

		int get_n_vertexes();
		std::vector<cv::Point*>* get_vertexes();
		cv::Point* get_vertex(int index);

	private:
		//Flags
		bool _flags[Mirror::_n_flags];

		//Mirror Mask
		cv::Mat1b _mask;

		//Mirror Plane
		ntk::Plane _plane;

		//Mirror Points
		std::vector<cv::Point*>* _points;
		std::vector<cv::Point*>* _user_points;
		
};

#endif