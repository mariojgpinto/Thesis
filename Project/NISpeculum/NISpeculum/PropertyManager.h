#ifndef _PROPERTY_MANAGER
#define _PROPERTY_MANAGER

class PropertyManager{
	public:
		static const int _n_capture_flags = 4;

		enum REQUESTS{
			R_REQUEST,
				R_FLOOR_AREA,
				R_FLOOR_POINTS,
				R_MIRROR_AREA,
				R_MIRROR_POINTS,
				R_SAVE,
				R_LOAD,
				R_SAVE_PCL,
				R_CAPTURE,
		};
		static const int _n_request_flags = 10;

		enum PROCESSED{
			P_FLOOR_PLANE = 1,
			P_MIRROR = 2,
			P_MASK = 4,
			P_POLYGON = 8,
			P_CAPTURE = 16,
			P_CAPTURE_SHOW = 32,
			P_FILTER = 64,
			P_OUTLIERS = 128,
			P_VOXEL = 256
		};
		static const int _n_processed_flags = 257;

		enum UPDATE{
			U_IMAGE,
			U_PCL,
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

		double _3d_normal_radius;

		bool _flag_requests[_n_request_flags];	
		bool _flag_processed[_n_processed_flags];
		bool _flag_update[_n_update_flags];

		//1 - Bilateral
		//2 - MedianBlur
		//3 - GaussinaBlur
		int	_filter_method;

		//1 - Radius
		//2 - Statistical
		int _outliers_methods;

		double	_voxel_grid_x;
		double	_voxel_grid_y;
		double	_voxel_grid_z;

		int		_outliers_radius_neighbors;
		double	_outliers_radius_radius;
		int		_outliers_statistical_meank;
		double	_outliers_statistical_stddev;


		//1 - pcd
		//2 - ply
		//3 - obj
		int				_save_pcl_mode;
		char			_file_name[256];
};

#endif//_PROPERTY_MANAGER