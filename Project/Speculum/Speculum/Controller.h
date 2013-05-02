/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QKinect.h>
#include <ntk/ntk.h>


//Windows
class Speculum;
class Preferences;
class MirrorManagerGUI;
class FloorManagerGUI;
class View3DWindow;
class Arena;


class View3DWindow;

class Controller{
public:
    Controller(QKinect* k);
    ~Controller();

    void set_viewer_window(Speculum* window){this->_main_window = window;}
	void set_preferences_window(Preferences* window){this->_preferences_window = window;}
	void set_mirror_manager_window(MirrorManagerGUI* window){this->_mirror_manager_window = window;}
	void set_floor_manager_window(FloorManagerGUI* window){this->_floor_manager_window = window;}
	void set_3d_window(View3DWindow* window){this->_view3d_window = window;}
    
    Speculum* get_viewer_window(){return this->_main_window;}
	Preferences* get_preferences_window(){return this->_preferences_window;}
	MirrorManagerGUI* get_mirror_manager_window(){return this->_mirror_manager_window;}
	FloorManagerGUI* get_floor_manager_window(){return this->_floor_manager_window;}
	View3DWindow* get_3d_window(){return this->_view3d_window;}
    
	void set_arena(Arena *arena){this->_arena = arena;}
	Arena* get_arena(){return this->_arena;}	
	
	QKinect* get_kinect(){return this->_kinect;} 

    //Global Images
    cv::Mat3b* get_color_image(){return this->_color_image;}
    cv::Mat1f* get_depth_image(){return this->_depth_image;}
    cv::Mat3b* get_depth_as_color(){return this->_depth_as_color;}
    cv::Mat1b* get_depth_mask(){return this->_depth_mask;}
    cv::Mat1b* get_diff(){return this->_diff;}
    ntk::RGBDImage* get_RGBDImage(){return this->_kinect_image;}

    //Global Variables
    void set_min_depth(double min){this->_min_depth = min;}
    void set_max_depth(double max){this->_max_depth = max;}

    double get_min_depth(){return this->_min_depth;}
    double get_max_depth(){return this->_max_depth;}

    //Generators
    xn::DepthGenerator& get_depth_generator(){return this->_kinect->get_depth_generator();}
    xn::ImageGenerator& get_rgb_generator(){return this->_kinect->get_rgb_generator();}
    //xn::IRGenerator& get_ir_generator(){return this->_kinect->niIRGenerator();}
    xn::UserGenerator& get_user_generator(){return this->_kinect->get_user_generator();}
	xn::Context& get_context(){return this->_kinect->get_context();}

    void update(ntk::RGBDImage* image);
	void update_timer();
    void process_images();
	void show_images();

	void set_paused(bool state){this->_paused = state;}
    bool is_paused(){return this->_paused;}

	//Buffer for 3DView
	void start_buffering(int n_frames);	
	void join_buffer();

private:
    void setup_images();
    void setup_variables();

private:
    //Windows
    Speculum* _main_window;
	Preferences* _preferences_window;
	MirrorManagerGUI* _mirror_manager_window;
	FloorManagerGUI* _floor_manager_window;
	View3DWindow* _view3d_window;

    //Global Variables
    QKinect *_kinect;
	Arena *_arena;

    //Global Images
    cv::Mat3b* _color_image;
    cv::Mat1f* _depth_image;
    cv::Mat3b* _depth_as_color;
    cv::Mat1b* _depth_mask;
    cv::Mat1b* _diff;
    ntk::RGBDImage* _kinect_image;
    ntk::RGBDImage* _kinect_image_temp;

    //Global Preferences
    double _min_depth;
    double _max_depth;

    bool _paused;

	//timer
	double _last_tick;
	int _frame_counter;
	float _frame_rate;

	//3D Buffering
	bool _buffer_flag;
	std::vector<ntk::RGBDImage*>* _buffer;
	int _buffer_counter;
	cv::Mat1b *_buffer_mask;
};

#endif // CONTROLLER_H
