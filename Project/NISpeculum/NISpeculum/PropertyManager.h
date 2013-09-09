#ifndef _PROPERTY_MANAGER
#define _PROPERTY_MANAGER

class PropertyManager{
	public:
		static const int _n_capture_flags = 4;

		enum REQUESTS{
			R_REQUEST,
				R_FLOOR,
				R_MIRROR_AREA,
				R_MIRROR_POINTS,
				R_SAVE,
				R_LOAD
		};
		static const int _n_request_flags = 6;

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
		bool _pause;
		int _3d_step;

		//Depth
		static const int depth_max = 2625;
		static const int depth_min = 400;
		int _depth_min;
		int _depth_max;	

		bool _flag_requests[_n_request_flags];	
		bool _flag_processed[_n_processed_flags];
		bool _flag_update[_n_update_flags];
};

#endif//_PROPERTY_MANAGER