#ifndef _FLOOR
#define _FLOOR 

#include <opencv2\opencv.hpp>

class Floor{
	public:
		enum FLAGS{
			MASK
		};
		static const int _n_flags = 1;


	public:
		Floor();
		~Floor();

		void enable_flag(Floor::FLAGS flag);
		void setup_variables();

	private:
		bool *_flags;

		cv::Mat1b _mask;
};

#endif