#ifndef POINTSELECTIONGUI_H
#define POINTSELECTIONGUI_H

#include <QtGui/QMainWindow>

namespace Ui {
	class PointSelectionGUI;
}

#include <opencv2\opencv.hpp>

#include <ToolBoxQT.h>

#ifdef _MY_UI_DLL
    #if (defined(QT_DLL) || defined(QT_SHARED)) && !defined(QT_PLUGIN)
        #define EXPORT Q_DECL_EXPORT
    #else
        #define EXPORT
    #endif
#else
        #define EXPORT Q_DECL_IMPORT
#endif 

class /*EXPORT*/ PointSelectionGUI : public QMainWindow
{
	Q_OBJECT

public:
	PointSelectionGUI(QWidget *parent = 0, Qt::WFlags flags = 0);
	~PointSelectionGUI();

	void set_image(cv::Mat &img);

	void get_points(std::vector<cv::Point*> &pts);

    void mousePressEvent(QMouseEvent *event);
	
public slots:
	void on_exit();
	void on_reset();

private:
	void setup_windows();
	void setup_connections();

public:
	std::vector<cv::Point>* _points;

	cv::Mat _image_orig;
	cv::Mat _image;

private:
	Ui::PointSelectionGUI* _ui;
	QAction* _close;

	ToolBoxQT::CVWidget* _widget;
};

#endif // POINTSELECTIONGUI_H