/**
 * Author: Mario Pinto
 *
 * 
 */

#include "Speculum.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	QKinect kinect("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers2.oni");
	//QKinect kinect("C:\\Dev\\Kinect\\Data\\ONI\\near_mode.oni");
	//QKinect kinect;
	//QKinect kinect("C:\\exp.oni");

	Controller controller(&kinect);

	Preferences preferences(&controller);
	MirrorManagerGUI mirror_manager(&controller, &a);
	FloorManagerGUI floor_manager(&controller, &a);
	View3DWindow view3d(&controller);
	Speculum speculum(&controller, &a);
	
	Arena arena;
	controller.set_arena(&arena);

	speculum.show();
	return a.exec();
}
