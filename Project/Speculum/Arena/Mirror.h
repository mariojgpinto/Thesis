/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef _MIRROR
#define _MIRROR

#include <opencv2\opencv.hpp>
#include <ToolBoxCV.h>

class Mirror{
	public:
		enum FLAGS{
			MASK
		};
		static const int _n_flags = 1;

	public:
		Mirror();
		~Mirror();

		void set_area(std::vector<cv::Point*>* points);

		void enable_flag(Mirror::FLAGS flag, bool value);
		bool check_flag(Mirror::FLAGS flag);
		void setup_variables();

		cv::Mat1b* get_mask(){return &this->_mask;}

	private:
		bool *_flags;

		cv::Mat1b _mask;
};

#endif