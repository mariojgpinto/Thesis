#include "MirrorManagerGUI.h"

#include "..\NISpeculum\Controller.h"
//#include "..\NISpeculum\PropertyManager.h"
#include "..\NISpeculum\Mirror.h"

#include "GUIController.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
MirrorManagerGUI::MirrorManagerGUI(Controller* controller, QApplication *app, QWidget *parent, Qt::WFlags flags): 
	QMainWindow(parent, flags),
	_q_application(app),
	_ui(new Ui::MirrorManager)
{
	_ui->setupUi(this);

	this->_controller = controller;

	this->_idx_mirror = -1;

	this->setup_windows();
	this->setup_connections();
}

MirrorManagerGUI::~MirrorManagerGUI()
{

}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void MirrorManagerGUI::setup_windows(){
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

	this->_cvwidget = new ToolBoxQT::CVWidget(this->_ui->_mirror_widget);
    this->_cvwidget->setObjectName(QString::fromUtf8("Color"));
    this->_cvwidget->setSizePolicy(sizePolicy1);
	this->_cvwidget->setFixedSize(this->_ui->_mirror_widget->width(),this->_ui->_mirror_widget->height());
}

void MirrorManagerGUI::setup_connections(){
	this->_ui->_mirror_action_close->setShortcut(Qt::Key_Escape);
	connect(this->_ui->_mirror_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	this->_ui->_mirror_push_button_add->setShortcut(QKeySequence("Ctrl+M"));
	connect(this->_ui->_mirror_push_button_add,SIGNAL(clicked()),this,SLOT(on_button_add_mirror()));

	connect(this->_ui->_mirror_check_box_color, SIGNAL(stateChanged(int)), this, SLOT(on_mirror_check_box_color(int)));
	connect(this->_ui->_mirror_spin_box_color_r, SIGNAL(valueChanged(int)), this, SLOT(on_mirror_spin_box_color_r(int)));
	connect(this->_ui->_mirror_spin_box_color_g, SIGNAL(valueChanged(int)), this, SLOT(on_mirror_spin_box_color_g(int)));
	connect(this->_ui->_mirror_spin_box_color_b, SIGNAL(valueChanged(int)), this, SLOT(on_mirror_spin_box_color_b(int)));

	connect(this->_ui->_mirror_horizontal_slider_a, SIGNAL(valueChanged(int)), this, SLOT(on_slider_a(int)));
	connect(this->_ui->_mirror_horizontal_slider_b, SIGNAL(valueChanged(int)), this, SLOT(on_slider_b(int)));
	connect(this->_ui->_mirror_horizontal_slider_c, SIGNAL(valueChanged(int)), this, SLOT(on_slider_c(int)));
	connect(this->_ui->_mirror_horizontal_slider_d, SIGNAL(valueChanged(int)), this, SLOT(on_slider_d(int)));

	connect(this->_ui->_mirror_spin_box, SIGNAL(valueChanged(int)), this, SLOT(on_mirror_spin_box(int)));

	connect(this->_ui->_mirror_horizontal_slider_min, SIGNAL(valueChanged(int)), this, SLOT(on_slider_min(int)));
	connect(this->_ui->_mirror_horizontal_slider_max, SIGNAL(valueChanged(int)), this, SLOT(on_slider_max(int)));
}

//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------
void MirrorManagerGUI::on_close(){
	this->hide();
}

void MirrorManagerGUI::on_button_add_mirror(){
	static bool area = true;

	if(area){
		this->_controller->_gui->point_selection(3);
		area = false;
	}
	else{
		this->_controller->_gui->point_selection(4);
		area = true;
	}	
}

void MirrorManagerGUI::on_mirror_check_box_color(int value){
	if(this->_idx_mirror >= 0){
		this->_controller->_mirrors[this->_idx_mirror]->_color = (value) ? true : false;
	}
}

void MirrorManagerGUI::on_mirror_spin_box_color_r(int value){
	if(this->_idx_mirror >= 0){
		this->_controller->_mirrors[this->_idx_mirror]->_color_r = value;
	}
}

void MirrorManagerGUI::on_mirror_spin_box_color_g(int value){
	if(this->_idx_mirror >= 0){
		this->_controller->_mirrors[this->_idx_mirror]->_color_g = value;
	}
}

void MirrorManagerGUI::on_mirror_spin_box_color_b(int value){
	if(this->_idx_mirror >= 0){
		this->_controller->_mirrors[this->_idx_mirror]->_color_b = value;
	}
}

void MirrorManagerGUI::on_slider_a(int value){
	ToolBox::Plane* plane = &this->_controller->_mirrors[this->_idx_mirror]->_plane;

	this->_controller->_mirrors[this->_idx_mirror]->_plane.set_normalized(value/1000.0,plane->_b,plane->_c,plane->_d);
	
	char buff[128];
	sprintf(buff,"A: %.4f\0",plane->_a);
	this->_ui->_mirror_label_a->setText(buff);
	sprintf(buff,"B: %.4f\0",plane->_b);
	this->_ui->_mirror_label_b->setText(buff);
	sprintf(buff,"C: %.4f\0",plane->_c);
	this->_ui->_mirror_label_c->setText(buff);
	sprintf(buff,"D: %.4f\0",plane->_d);
	this->_ui->_mirror_label_d->setText(buff);
}

void MirrorManagerGUI::on_slider_b(int value){
	ToolBox::Plane* plane = &this->_controller->_mirrors[this->_idx_mirror]->_plane;

	this->_controller->_mirrors[this->_idx_mirror]->_plane.set_normalized(plane->_a,value/1000.0,plane->_c,plane->_d);
	
	char buff[128];
	sprintf(buff,"A: %.4f\0",plane->_a);
	this->_ui->_mirror_label_a->setText(buff);
	sprintf(buff,"B: %.4f\0",plane->_b);
	this->_ui->_mirror_label_b->setText(buff);
	sprintf(buff,"C: %.4f\0",plane->_c);
	this->_ui->_mirror_label_c->setText(buff);
	sprintf(buff,"D: %.4f\0",plane->_d);
	this->_ui->_mirror_label_d->setText(buff);
}

void MirrorManagerGUI::on_slider_c(int value){
	ToolBox::Plane* plane = &this->_controller->_mirrors[this->_idx_mirror]->_plane;

	this->_controller->_mirrors[this->_idx_mirror]->_plane.set_normalized(plane->_a,plane->_b,value/1000.0,plane->_d);
	
	char buff[128];
	sprintf(buff,"A: %.4f\0",plane->_a);
	this->_ui->_mirror_label_a->setText(buff);
	sprintf(buff,"B: %.4f\0",plane->_b);
	this->_ui->_mirror_label_b->setText(buff);
	sprintf(buff,"C: %.4f\0",plane->_c);
	this->_ui->_mirror_label_c->setText(buff);
	sprintf(buff,"D: %.4f\0",plane->_d);
	this->_ui->_mirror_label_d->setText(buff);
}

void MirrorManagerGUI::on_slider_d(int value){
	if(value < 0) return;
	ToolBox::Plane* plane = &this->_controller->_mirrors[this->_idx_mirror]->_plane;

	this->_controller->_mirrors[this->_idx_mirror]->_plane.set_normalized(plane->_a,plane->_b,plane->_c,value*-1);
	
	char buff[128];
	sprintf(buff,"A: %.4f\0",plane->_a);
	this->_ui->_mirror_label_a->setText(buff);
	sprintf(buff,"B: %.4f\0",plane->_b);
	this->_ui->_mirror_label_b->setText(buff);
	sprintf(buff,"C: %.4f\0",plane->_c);
	this->_ui->_mirror_label_c->setText(buff);
	sprintf(buff,"D: %.4f\0",plane->_d);
	this->_ui->_mirror_label_d->setText(buff);
}

void MirrorManagerGUI::on_mirror_spin_box(int value){
	this->_idx_mirror = value;
	this->update_values();
}

void MirrorManagerGUI::on_slider_min(int value){
	if(this->_controller->_mirrors.size()){
		this->_controller->_mirrors[this->_idx_mirror]->_depth_min = value;
	}
	char buff[128];
	sprintf(buff,"Min: %dmm",value);
	this->_ui->_mirror_label_min->setText(buff);
}

void MirrorManagerGUI::on_slider_max(int value){
	if(this->_controller->_mirrors.size()){
		this->_controller->_mirrors[this->_idx_mirror]->_depth_max = value;
	}
	char buff[128];
	sprintf(buff,"Max: %dmm",value);
	this->_ui->_mirror_label_max->setText(buff);
}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
void MirrorManagerGUI::update_values(){
	if(this->_controller->_mirrors.size()){
		this->_idx_mirror = (this->_idx_mirror == -1) ? this->_controller->_mirrors.size() -1 : this->_idx_mirror;

		this->_ui->_mirror_check_box_color->setChecked(this->_controller->_mirrors[this->_idx_mirror]->_color);
		this->_ui->_mirror_spin_box_color_r->setValue(this->_controller->_mirrors[this->_idx_mirror]->_color_r);
		this->_ui->_mirror_spin_box_color_g->setValue(this->_controller->_mirrors[this->_idx_mirror]->_color_g);
		this->_ui->_mirror_spin_box_color_b->setValue(this->_controller->_mirrors[this->_idx_mirror]->_color_b);

		this->_ui->_mirror_spin_box->setMaximum(this->_controller->_mirrors.size() -1);
		this->_ui->_mirror_spin_box->setMinimum(0);

		ToolBox::Plane* plane = &this->_controller->_mirrors[this->_idx_mirror]->_plane;

		this->_ui->_mirror_horizontal_slider_a->setValue(plane->_a * 1000);
		this->on_slider_a(plane->_a * 1000);
		this->_ui->_mirror_horizontal_slider_b->setValue(plane->_b * 1000);
		this->on_slider_b(plane->_b * 1000);
		this->_ui->_mirror_horizontal_slider_c->setValue(plane->_c * 1000);
		this->on_slider_c(plane->_c * 1000);
		this->_ui->_mirror_horizontal_slider_d->setValue(plane->_d * -1);
		this->on_slider_d(plane->_d * 1000);

		this->_ui->_mirror_horizontal_slider_min->setValue(this->_controller->_mirrors[this->_idx_mirror]->_depth_min);
		this->_ui->_mirror_horizontal_slider_max->setValue(this->_controller->_mirrors[this->_idx_mirror]->_depth_max);
	}
}

void MirrorManagerGUI::update_widget(){
	if(this->_idx_mirror >= 0){
		cv::Mat temp;
		this->_controller->_mat_color_bgr.copyTo(temp,this->_controller->_mirrors[this->_idx_mirror]->_area_mask);
		this->_cvwidget->setImage(&temp);
	}
}