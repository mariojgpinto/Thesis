#include "Controller.h"

int main(int argc, char* argv[]){

	//SpeculumGUI *gui = new SpeculumGUI(argc,argv);
	//NIKinect* kinect = new NIKinect();

	Controller* controller = new Controller();

	controller->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");

	controller->run(argc, argv);


	return 0;
}