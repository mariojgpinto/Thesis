/**
 * Author: Mario Pinto
 *
 * 
 */

#include "FloorManagerGUI.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
FloorManagerGUI::FloorManagerGUI(Controller *c, QApplication *a, QWidget *parent, Qt::WFlags flags):
	QMainWindow(parent, flags),
	app(a),
	ui(new Ui::FloorManagerGUI)
{
	ui->setupUi(this);

	this->_controller = c;
    this->_controller->set_floor_manager_window(this);

	this->_controller->set_paused(true);

    this->setup_windows();
    this->setup_connections();

    this->_controller->set_paused(false);
}

FloorManagerGUI::~FloorManagerGUI()
{
	delete ui;
}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void FloorManagerGUI::setup_windows(){
 //   QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
 //   sizePolicy1.setHorizontalStretch(0);
 //   sizePolicy1.setVerticalStretch(0);
 //   sizePolicy1.setHeightForWidth(true);

	//this->_ntk_widget_top = new ntk::ImageWidget(this->ui->_mirror_widget_top);
 //   this->_ntk_widget_top->setObjectName(QString::fromUtf8("mirror_widget_top"));
 //   this->_ntk_widget_top->setSizePolicy(sizePolicy1);
	//this->_ntk_widget_top->setFixedSize(this->ui->_mirror_widget_top->width(),
	//									this->ui->_mirror_widget_top->height());

	//this->_ntk_widget_bottom_left = new ntk::ImageWidget(this->ui->_mirror_widget_bottom_left);
 //   this->_ntk_widget_bottom_left->setObjectName(QString::fromUtf8("mirror_widget_bottom_left"));
 //   this->_ntk_widget_bottom_left->setSizePolicy(sizePolicy1);
	//this->_ntk_widget_bottom_left->setFixedSize(this->ui->_mirror_widget_bottom_left->width(),
	//											this->ui->_mirror_widget_bottom_left->height());

	//this->_ntk_widget_bottom_right = new ntk::ImageWidget(this->ui->_mirror_widget_bottom_right);
 //   this->_ntk_widget_bottom_right->setObjectName(QString::fromUtf8("mirror_widget_bottom_right"));
 //   this->_ntk_widget_bottom_right->setSizePolicy(sizePolicy1);
	//this->_ntk_widget_bottom_right->setFixedSize(	this->ui->_mirror_widget_bottom_right->width(),
	//												this->ui->_mirror_widget_bottom_right->height());

    //int _min = (this->_controller->get_min_depth() - this->min_value) * 100 / this->max_value;
    //this->min_slider_change(_min);
    //this->ui->main_horizontalSlider_min_depth->setValue(_min);
    //int _max = (this->_controller->get_max_depth() - this->min_value) * 100 / this->max_value;
    //this->max_slider_change(_max);
    //this->ui->main_horizontalSlider_max_depth->setValue(_max);
}

void FloorManagerGUI::setup_connections(){
 	this->ui->_floor_action_close->setShortcut(Qt::Key_Escape);
	connect(this->ui->_floor_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	
    //connect(this->ui->main_pushButton_background, SIGNAL(clicked()), this, SLOT(on_botton_background()));
    //connect(this->ui->main_pushButton_user, SIGNAL(clicked()), this, SLOT(on_botton_user()));
    //connect(this->ui->main_horizontalSlider_min_depth, SIGNAL(valueChanged(int)), this, SLOT(min_slider_change(int)));
    //connect(this->ui->main_horizontalSlider_max_depth, SIGNAL(valueChanged(int)), this, SLOT(max_slider_change(int)));
}

//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// SLOTS - FILE ACTIONS 
//-----------------------------------------------------------------------------
void FloorManagerGUI::on_close(){
	this->hide();
}


//-----------------------------------------------------------------------------
// Processing
//-----------------------------------------------------------------------------
void FloorManagerGUI::process_image()
{
	//this->_controller->process_images();
    //if(!this->_controller->get_user_window()->isHidden()){
    //    this->_controller->get_user_window()->process_image();
    //}
}

void FloorManagerGUI::show_images()
{
	//this->_ntk_widget_top->setImage(*this->_controller->get_depth_as_color());
	//this->_ntk_widget_bottom_left->setImage(*this->_controller->get_color_image());
	//this->_ntk_widget_bottom_right->setImage(*this->_controller->get_depth_image());
}
