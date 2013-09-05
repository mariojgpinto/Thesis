#ifndef _PROPERTY_MANAGER
#define _PROPERTY_MANAGER

class PropertyManager{
	public:
		enum CAPTURE{
			DEPTH		= 1,	/**< Capture and copy the Depth cv::Mat.*/
			DEPTH_MD	= 2,	/**< Capture and copy the Depth MetaData.*/
			COLOR		= 4,	/**< Capture and copy the Color cv::Mat.*/
			COLOR_MD	= 8		/**< Capture and copy the Color MetaData.*/
		};
		static const int _n_capture_flags = 4;

		enum REQUESTS{
			R_REQUEST,
				R_FLOOR,
				R_MIRROR,
				R_SAVE,
				R_LOAD
		};
		static const int _n_request_flags = 5;

		enum PROCESSED{
			P_FLOOR_PLANE,
			P_MIRROR,
			P_MASK
		};
		static const int _n_processed_flags = 3;

		enum UPDATE{
			U_IMAGE,
			U_PCL
		};
		static const int _n_update_flags = 2;

	public:
		PropertyManager();
		~PropertyManager();

	public:
		bool _running;
		int _3d_step;

		//Depth
		static const int depth_max = 2625;
		static const int depth_min = 400;
		int _depth_min;
		int _depth_max;	

		bool _flag_capture[_n_capture_flags * _n_capture_flags + _n_capture_flags + 1];	
		bool _flag_requests[_n_request_flags];	
		bool _flag_processed[_n_processed_flags];
		bool _flag_update[_n_update_flags];
};

#endif//_PROPERTY_MANAGER