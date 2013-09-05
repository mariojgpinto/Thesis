/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef _ARENA
#define _ARENA

#include <Windows.h>

#include "Mirror.h"
#include "Floor.h"

class __declspec(dllexport)Arena{
	public:
		enum FLAGS{
			MIRRORS,
			FLOOR
		};
		static const int _n_flags = 2;

	public:
		Arena();
		~Arena();

		//Setup
		void setup_variables();

		//Flag
		void enable_flag(Arena::FLAGS flag, bool value);
		bool check_flag(Arena::FLAGS flag);

		//Configuration
		void add_mirror(Mirror* mirror);
		void add_floor(Floor* floor);

		//Processing
		//ntk::Mesh* calculate_mesh(ntk::RGBDImage* image);
		
		//Access
		std::vector<Mirror*>* get_mirrors();
		Mirror* get_mirror(int index);
		int get_n_mirrors();
		Floor* get_floor();

		cv::Mat* get_floor_mask();
		cv::Mat* get_mirror_mask(int idx);
		cv::Mat* get_mirror_masks();
		cv::Mat* get_all_mask();

		//Storage
		bool save_to_file(char* path = 0, char* filename = 0);
		bool load_from_file(char* file_path);

	//UTIL
	public:
		//Processing
		//static ntk::Plane* extract_plane(std::vector<cv::Point*>* points, ntk::RGBDImage* image, int point_radius = 1);

	//Variables
	public:
		bool _flags[_n_flags];

		//Mirrors
		std::vector<Mirror*>* _mirrors;

		//Floor
		Floor* _floor;
};

#endif