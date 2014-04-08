#include "Controller.h"

int main(int argc, char* argv[]){

	//SpeculumGUI *gui = new SpeculumGUI(argc,argv);
	//NIKinect* kinect = new NIKinect();

	Controller* controller = new Controller();

	if(argc == 2){
		controller->init(argv[1]);
	}
	else{
		controller->init();
	}
	//controller->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");
	//controller->init("C:\\Dev\\Walkys\\Project\\Data\\Mirrors\\mirror_calib.oni");
	//controller->init("C:\\Dev\\Walkys\\Project\\Data\\Mirrors\\mirror_mirror_boxes.oni");
	//controller->init("C:\\Dev\\Walkys\\Project\\Data\\Mirrors\\mirror_mirror_legs.oni");
	controller->run(argc, argv);


	return 0;
}