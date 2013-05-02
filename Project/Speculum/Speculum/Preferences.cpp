/**
 * Author: Mario Pinto
 *
 * 
 */

#include "Preferences.h"

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
Preferences::Preferences(Controller *c, QWidget *parent, Qt::WFlags flags):
	_min_value(0.4),_max_value(2.0),
	QMainWindow(parent, flags),
	ui(new Ui::Preferences),
	_controller(c)
{
	ui->setupUi(this);

	this->_controller->set_preferences_window(this);

	this->setup_windows();
    this->setup_connections();
}

Preferences::~Preferences()
{
}

    
//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/** 
 *
 */
void Preferences::setup_windows(){
	QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
	sizePolicy1.setHorizontalStretch(0);
	sizePolicy1.setVerticalStretch(0);
	sizePolicy1.setHeightForWidth(true);

	this->_widget = new ntk::ImageWidget(this->ui->_preferences_widget);
	this->_widget->setObjectName(QString::fromUtf8("DepthAsColor"));
	this->_widget->setSizePolicy(sizePolicy1);
	this->_widget->setFixedSize(this->ui->_preferences_widget->width(),this->ui->_preferences_widget->height());
}

/** 
 *
 */
void Preferences::setup_connections(){
	this->_close = new QAction(this);
	this->_close->setShortcut(Qt::Key_Escape);
	connect(this->_close,SIGNAL(triggered()),this,SLOT(on_close()));
	addAction(this->_close);
	//connect(this->ui->main_pushButton_user, SIGNAL(clicked()), this, SLOT(on_botton_user()));
    
	connect(this->ui->preferences_slider_min_depth, SIGNAL(valueChanged(int)), this, SLOT(min_slider_change(int)));
	connect(this->ui->preferences_slider_max_depth, SIGNAL(valueChanged(int)), this, SLOT(max_slider_change(int)));
}

void Preferences::setup_variables(){
	int _min = (this->_controller->get_min_depth() - this->_min_value) * 100 / this->_max_value;
    this->min_slider_change(_min);
    this->ui->preferences_slider_min_depth->setValue(_min);
    int _max = (this->_controller->get_max_depth() - this->_min_value) * 100 / this->_max_value;
    this->max_slider_change(_max);
    this->ui->preferences_slider_max_depth->setValue(_max);
}


//-----------------------------------------------------------------------------
// SLOTS
//-----------------------------------------------------------------------------
/**
 *
 */
void Preferences::on_close(){
	cv::imwrite("image.png",*this->_controller->get_depth_as_color());

	cv::Mat coiso;//(this->_controller->get_depth_as_color()->size());

	cv::cvtColor(*this->_controller->get_depth_as_color(), coiso, CV_RGB2GRAY);
	cv::imwrite("image_gray.png",*this->_controller->get_depth_image());
	cv::Mat show; //this->_controller->get_depth_image()->convertTo( show, CV_32SC1, 0.05 );
	cv::blur(*this->_controller->get_depth_image(),show,cv::Size(3,3));
	cv::FileStorage fs("test.yml", cv::FileStorage::WRITE);
	fs << "cameraMatrix" << *this->_controller->get_depth_image();
	fs.release();

	cv::FileStorage fs1("test2.yml", cv::FileStorage::WRITE);
	fs1 << "cameraMatrix" << show;
	fs1.release();

	cv::imshow("show",show);
	cv::waitKey();
	cv::imwrite("image_gray_orig.png",show);

	this->hide();
}

/**
 *
 */
void Preferences::min_slider_change(int value){
    double read_value = value/100.0;

    this->_controller->set_min_depth(this->_min_value + read_value * this->_max_value);
	
    this->ui->preferences_label_min_depth->setText(QString("Min Depth: %1 m").arg(this->_controller->get_min_depth(), 0, 'f', 2));

    if(this->ui->preferences_check_box_diff->isChecked()){
        double diff = value - this->_min_depth_old;

        this->_controller->set_max_depth(this->_controller->get_max_depth() + diff/100.0);
        this->ui->preferences_slider_max_depth->setValue(this->_max_depth_old + diff);
        this->ui->preferences_label_max_depth->setText(QString("Max Depth: %1 m").arg(this->_controller->get_max_depth(), 0, 'f', 2));
    }

    this->_min_depth_old = value;
}

/**
 *
 */
void Preferences::max_slider_change(int value){
    double read_value = value/100.0;

    this->_controller->set_max_depth(this->_min_value + read_value * this->_max_value);

    this->ui->preferences_label_max_depth->setText(QString("Max Depth: %1 m").arg(this->_controller->get_max_depth(), 0, 'f', 2));

    if(this->ui->preferences_check_box_diff->isChecked()){
        double diff = value - this->_max_depth_old;

        this->_controller->set_min_depth(this->_controller->get_min_depth() + diff/100.0);
        this->ui->preferences_slider_min_depth->setValue(this->_min_depth_old + diff);
        this->ui->preferences_label_min_depth->setText(QString("Min Depth: %1 m").arg(this->_controller->get_min_depth(), 0, 'f', 2));
    }

    this->_max_depth_old = value;
}

//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
/** 
 *
 */
void Preferences::process_image(){

}


//-----------------------------------------------------------------------------
// SHOW
//-----------------------------------------------------------------------------
/** 
 *
 */
void Preferences::show_images(){
	this->_widget->setImage(*this->_controller->get_depth_as_color());
}