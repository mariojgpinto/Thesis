#include "PointSelectionGUI.h"

#include "ui_PointSelectionGUI.h"

#include <QMouseEvent>

//#include <ToolBoxCV.h>

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
PointSelectionGUI::PointSelectionGUI(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags),
	_ui(new Ui::PointSelectionGUI)
{
	_ui->setupUi(this);
	
	this->_points = new std::vector<cv::Point>();

	this->setup_windows();
	this->setup_connections();

	this->_widget->setImage(&this->_image);
}

PointSelectionGUI::~PointSelectionGUI()
{

}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
void PointSelectionGUI::setup_windows(){
	QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

	this->_widget = new ToolBoxQT::CVWidget(this->_ui->_widget);
    this->_widget->setObjectName(QString::fromUtf8("Widget"));
    this->_widget->setSizePolicy(sizePolicy1);
	this->_widget->setFixedSize(this->_ui->_widget->width(),this->_ui->_widget->height());
}

void PointSelectionGUI::setup_connections(){
	this->_ui->_pushButton_exit->setShortcut(Qt::Key_Escape);
	connect(this->_ui->_pushButton_exit, SIGNAL(clicked()), this, SLOT(on_exit()));
	connect(this->_ui->_pushButton_reset, SIGNAL(clicked()), this, SLOT(on_reset()));
}

//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------
void PointSelectionGUI::on_exit(){
	this->hide();
}

void PointSelectionGUI::on_reset(){
	this->_image_orig.copyTo(this->_image);

	this->_points->clear();
	this->_widget->setImage(&this->_image);
}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
void PointSelectionGUI::mousePressEvent(QMouseEvent *event){
	QPoint pos = this->_ui->_widget->mapFrom(this->_ui->centralWidget,event->pos());


	if(pos.x() > 0 && pos.y() > 0 && pos.x() < this->_image_orig.cols && pos.y() < this->_image_orig.rows){
		cv::circle(this->_image,cv::Point(pos.x(),pos.y()),5,cv::Scalar(0,0,255),-1);
		this->_points->push_back(cv::Point(pos.x(),pos.y()));
	}

	this->_widget->setImage(&this->_image);
}

void PointSelectionGUI::set_image(cv::Mat &img){
	if(img.cols && img.rows){
		img.copyTo(this->_image_orig);
		
		this->on_reset();
	}
}

//-----------------------------------------------------------------------------
// INFO
//-----------------------------------------------------------------------------
void PointSelectionGUI::get_points(std::vector<cv::Point*> &pts){
	pts.clear();

	for(int i = 0 ; i < this->_points->size() ; ++i){
		pts.push_back(new cv::Point(this->_points->at(i)));
	}
}