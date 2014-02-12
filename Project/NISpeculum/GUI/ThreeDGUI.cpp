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

	this->on_spin_box_nframes(10);
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

	

	connect(this->_ui->_3d_double_spin_box_normal, SIGNAL(valueChanged(double)), this, SLOT(on_spin_box_radius_normal(double)));
	
	connect(this->_ui->_3d_check_box_filter, SIGNAL(stateChanged(int)), this, SLOT(on_check_box_filter(int)));
	connect(this->_ui->_3d_combo_box_filter, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_combo_box_filter(QString)));
	connect(this->_ui->_3d_check_box_outliers, SIGNAL(stateChanged(int)), this, SLOT(on_check_box_outliers(int)));
			
	connect(this->_ui->_3d_combo_box_filetype, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_combo_box_file_type(QString)));
	connect(this->_ui->_3d_push_button_save,SIGNAL(clicked()),this,SLOT(on_button_save()));

	connect(this->_ui->_3d_push_button_capture,SIGNAL(clicked()),this,SLOT(on_button_capture_model()));
	connect(this->_ui->_3d_spin_box_nframes, SIGNAL(valueChanged(int)), this, SLOT(on_spin_box_nframes(int)));

	connect(this->_ui->_3d_check_box_voxel, SIGNAL(stateChanged(int)), this, SLOT(on_check_box_voxel(int)));
	connect(this->_ui->_3d_double_spin_box_voxel_x, SIGNAL(valueChanged(double)), this, SLOT(on_spin_box_voxel_x(double)));
	connect(this->_ui->_3d_double_spin_box_voxel_y, SIGNAL(valueChanged(double)), this, SLOT(on_spin_box_voxel_y(double)));
	connect(this->_ui->_3d_double_spin_box_voxel_z, SIGNAL(valueChanged(double)), this, SLOT(on_spin_box_voxel_z(double)));

	connect(this->_ui->_3d_combo_box_filter_3d, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_combo_box_filter_3d(QString)));
	connect(this->_ui->_3d_spin_box_minneighbors, SIGNAL(valueChanged(int)), this, SLOT(on_spin_box_minneighbors(int)));
	connect(this->_ui->_3d_double_spin_box_radius, SIGNAL(valueChanged(double)), this, SLOT(on_spin_box_radius(double)));
	connect(this->_ui->_3d_spin_box_meank, SIGNAL(valueChanged(int)), this, SLOT(on_spin_box_meank(int)));
	connect(this->_ui->_3d_double_spin_box_stddev, SIGNAL(valueChanged(double)), this, SLOT(on_spin_box_stddev(double)));
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
	this->on_combo_box_file_type("");
	
	if(this->_ui->_3d_text_edit_file_name->toPlainText().length()){
		sprintf(this->_controller->_property_manager->_file_name,"%s",this->_ui->_3d_text_edit_file_name->toPlainText().toStdString().data());
	}
	else{
		sprintf(this->_controller->_property_manager->_file_name,"point_cloud");
	}

	this->_controller->_property_manager->_flag_requests[PropertyManager::R_REQUEST] = true;
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_SAVE_PCL] = true;
}

void ThreeDGUI::on_combo_box_file_type(QString value){
	if(this->_ui->_3d_combo_box_filetype->currentText().compare(".pcd") == 0){
		this->_controller->_property_manager->_save_pcl_mode = 1;
	} else
	if(this->_ui->_3d_combo_box_filetype->currentText().compare(".ply") == 0){
		this->_controller->_property_manager->_save_pcl_mode = 2;
	} else
	if(this->_ui->_3d_combo_box_filetype->currentText().compare(".obj") == 0){
		this->_controller->_property_manager->_save_pcl_mode = 3;
	}
}

void ThreeDGUI::on_spin_box_radius_normal(double value){
	this->_controller->_property_manager->_3d_normal_radius = value;
}

void ThreeDGUI::on_spin_box_nframes(int value){
	this->_controller->_model_n_frames = value;
}

void ThreeDGUI::on_button_capture_model(){
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_REQUEST] = true;
	this->_controller->_property_manager->_flag_requests[PropertyManager::R_CAPTURE] = true;


	//cv::namedWindow("win1");
	//cv::namedWindow("win2");
	//cv::namedWindow("win3");
	//cv::namedWindow("win4");
}


void ThreeDGUI::on_check_box_filter(int value){
	if(!value)
		this->_controller->_property_manager->_filter_method = 0;
	else
		this->on_combo_box_filter("");

	this->_controller->_property_manager->_flag_processed[PropertyManager::P_FILTER] = (value) ? true : false;
}

void ThreeDGUI::on_combo_box_filter(QString value){
	if(this->_ui->_3d_combo_box_filter->currentText().compare("Bilateral") == 0){
		this->_controller->_property_manager->_filter_method = 1;
	}else
	if(this->_ui->_3d_combo_box_filter->currentText().compare("MedianBlur") == 0){
		this->_controller->_property_manager->_filter_method = 2;
	} else
	if(this->_ui->_3d_combo_box_filter->currentText().compare("GaussianBlur") == 0){
		this->_controller->_property_manager->_filter_method = 3;
	}
	

}

void ThreeDGUI::on_check_box_outliers(int value){
	this->_controller->_property_manager->_flag_processed[PropertyManager::P_OUTLIERS] = (value) ? true : false;
}


void ThreeDGUI::on_check_box_voxel(int value){
	if(value){
		this->_controller->_property_manager->_voxel_grid_x = this->_ui->_3d_double_spin_box_voxel_x->value();
		this->_controller->_property_manager->_voxel_grid_y = this->_ui->_3d_double_spin_box_voxel_y->value();
		this->_controller->_property_manager->_voxel_grid_z = this->_ui->_3d_double_spin_box_voxel_z->value();

		this->_controller->_property_manager->_flag_processed[PropertyManager::P_VOXEL] = true;
	}
	else{
		this->_controller->_property_manager->_flag_processed[PropertyManager::P_VOXEL] = false;
	}
}

void ThreeDGUI::on_spin_box_voxel_x(double value){
	this->_controller->_property_manager->_voxel_grid_x = this->_ui->_3d_double_spin_box_voxel_x->value();
}

void ThreeDGUI::on_spin_box_voxel_y(double value){
	this->_controller->_property_manager->_voxel_grid_y = this->_ui->_3d_double_spin_box_voxel_y->value();
}

void ThreeDGUI::on_spin_box_voxel_z(double value){
	this->_controller->_property_manager->_voxel_grid_z = this->_ui->_3d_double_spin_box_voxel_z->value();
}



void ThreeDGUI::on_combo_box_filter_3d(QString value){
	if(this->_ui->_3d_combo_box_filter_3d->currentText().compare("Radius") == 0){
		this->_controller->_property_manager->_outliers_methods = 1;
	} else
	if(this->_ui->_3d_combo_box_filter_3d->currentText().compare("Statistical") == 0){
		this->_controller->_property_manager->_outliers_methods = 2;
	} 		
}

void ThreeDGUI::on_spin_box_minneighbors(int value){
	//_3d_spin_box_minneighbors
	this->_controller->_property_manager->_outliers_radius_neighbors = value;
		
}

void ThreeDGUI::on_spin_box_radius(double value){
	//_3d_double_spin_box_radius
	this->_controller->_property_manager->_outliers_radius_radius = value;
		
}

void ThreeDGUI::on_spin_box_meank(int value){
	//_3d_spin_box_meank
	this->_controller->_property_manager->_outliers_statistical_meank = value;
		
}

void ThreeDGUI::on_spin_box_stddev(double value){
	//_3d_double_spin_box_stddev
	this->_controller->_property_manager->_outliers_statistical_stddev = value;
}