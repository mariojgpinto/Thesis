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

	QKinect kinect("C:\\Dev\\Kinect\\Data\\ONI\\mirror_2_box.oni");
	//QKinect kinect("C:\\x.oni");

	Controller controller(&kinect);

	Preferences preferences(&controller);
	Speculum speculum(&controller, &a);

	speculum.show();
	return a.exec();
}
