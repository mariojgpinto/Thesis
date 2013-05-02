#include "3DView.h"

#include "ui_3DView.h"

#include "Controller.h"
#include <Arena.h>

#include <ntk/mesh/mesh_generator.h>

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
View3DWindow::View3DWindow(Controller *c, QWidget *parent) :
    QMainWindow(parent),    
    ui(new Ui::View3D),
	_n_frames(10)
{  
	ui->setupUi(this);

	_controller = c;
	this->_controller->set_3d_window(this);
	
	_mesh_generator = new ntk::MeshGenerator();
	_mesh_generator->setUseColor(true);

	ui->mesh_view->setBackgroundColor(cv::Vec4b(255,255,255,0));
	ui->mesh_view->enableLighting();

	this->setup_connections();
}

View3DWindow::~View3DWindow(){
	delete ui;
}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void View3DWindow::setup_connections(){
	this->ui->view3d_action_close->setShortcut(Qt::Key_Escape);
	connect(this->ui->view3d_action_close,SIGNAL(triggered()),this,SLOT(on_close()));

	connect(this->ui->_view3d_pushButton_capture, SIGNAL(clicked()), this, SLOT(on_capture_model()));
	connect(this->ui->_view3d_pushButton_capture_single, SIGNAL(clicked()), this, SLOT(on_capture_model_single_frame()));

	connect(this->ui->_view3d_spinBox_nframes, SIGNAL(valueChanged(int)), this, SLOT(on_n_frames(int)));
}


//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SLOTS - FILE ACTIONS 
//-----------------------------------------------------------------------------
void View3DWindow::on_close(){
	this->hide();
}

//-----------------------------------------------------------------------------
// SLOTS - CHECKBOX
//-----------------------------------------------------------------------------
void View3DWindow::on_resetCamera_clicked(){
  ui->mesh_view->resetCamera();
  ui->mesh_view->updateGL();
}

void View3DWindow::on_colorMappingCheckBox_toggled(bool checked){
	this->_mesh_generator->setUseColor(checked);
}

void View3DWindow::on_pointCloudPushButton_clicked(){
	this->_mesh_generator->setMeshType(ntk::MeshGenerator::PointCloudMesh);
}

void View3DWindow::on_surfelsPushButton_clicked(){
	this->_mesh_generator->setMeshType(ntk::MeshGenerator::SurfelsMesh);
}

void View3DWindow::on_trianglePushButton_clicked(){
	this->_mesh_generator->setMeshType(ntk::MeshGenerator::TriangleMesh);
	//if(this->_mesh_temp)
	//	this->_mesh_temp->saveToPlyFile("current_mesh.ply");
}

void View3DWindow::on_saveMeshPushButton_clicked(){
	if(this->_mesh_temp){
		char buff[256];
		SYSTEMTIME time;
		GetSystemTime(&time);
		sprintf(buff,"mesh_%04d%02d%02d_%02d%02d%02d.ply",time.wYear,time.wMonth,time.wDay,time.wHour,time.wMinute,time.wSecond);
		this->_mesh_temp->saveToPlyFile(buff);
	}
}


//-----------------------------------------------------------------------------
// MY BUTTONS
//-----------------------------------------------------------------------------
void View3DWindow::on_capture_model(){
	this->_controller->start_buffering(this->_n_frames);
	//this->_mesh_temp = this->_controller->get_arena()->calculate_mesh(this->_controller->get_RGBDImage());
}

void View3DWindow::on_capture_model_single_frame(){
	this->_mesh_temp = this->_controller->get_arena()->calculate_mesh(this->_controller->get_RGBDImage());
}

void View3DWindow::on_n_frames(int value){
	this->_n_frames = value;
}

//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
void View3DWindow::add_mesh(ntk::Mesh &mesh){
	this->_mesh_temp = &mesh;
	this->ui->mesh_view->addMesh(mesh, ntk::Pose3D(), ntk::MeshViewer::FLAT);
	this->ui->mesh_view->swapScene();
}	