/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef _ARENA
#define _ARENA

#include "Mirror.h"
#include "Floor.h"

class Arena{
	public:
		enum FLAGS{
			MASK
		};
		static const int _n_flags = 1;

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

		//Access
		std::vector<Mirror*>* get_mirrors();
		Mirror* get_mirror(int index);
		Floor* get_floor();


	//Variables
	private:
		bool *_flags;

		std::vector<Mirror*>* _mirrors;

		Floor* _floor;
};

#endif