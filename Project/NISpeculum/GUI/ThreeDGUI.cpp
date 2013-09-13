#include "ThreeDGUI.h"

#include "..\NISpeculum\Controller.h"
#include "..\NISpeculum\PropertyManager.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
ThreeDGUI::ThreeDGUI(Controller* controller, QApplication *app, QWidget *parent, Qt::WFlags flags): 
	QMainWindow(parent, flags),
	_q_application(app),
	_ui(new Ui::ThreeDGUI)
{
	_ui->setupUi(this);

	this->_controller = controller;

	this->setup_connections();
}

ThreeDGUI::~ThreeDGUI()
{

}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void ThreeDGUI::setup_connections(){
	this->_ui->_action_close->setShortcut(Qt::Key_Escape);
	connect(this->_ui->_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	connect(this->_ui->_3d_check_box_polygon, SIGNAL(stateChanged(int)), this, SLOT(on_check_box_polygon(int)));

	connect(this->_ui->_3d_push_button_save,SIGNAL(clicked()),this,SLOT(on_button_save()));
}

//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------
void ThreeDGUI::on_close(){
	this->hide();
}

void ThreeDGUI::on_check_box_polygon(int value){
	this->_controller->_property_manager->_flag_processed[PropertyManager::P_POLYGON] = (value != 0);
}

void ThreeDGUI::on_button_save(){
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_SAVE_PCL] = true;
}