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

		enum PROCESSING{

		};
		static const int _n_processing_flags = 2;

		enum VIEWER{

		};
		static const int _n_viewer_flags = 2;



	public:
		PropertyManager();
		~PropertyManager();

		bool _running;
		int _3d_step;
	private:
		bool _flag_capture[_n_capture_flags * _n_capture_flags + _n_capture_flags + 1];

		

		
};

#endif//_PROPERTY_MANAGER