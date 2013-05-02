/**
 * Author: Mario Pinto
 *
 * 
 */

#ifndef VIEW3DWINDOW_H
#define VIEW3DWINDOW_H

#include <QMainWindow>

namespace Ui {
    class View3D;
}

namespace ntk {
    class Mesh;
	class MeshGenerator;
}

class Controller;

class View3DWindow : public QMainWindow
{
    Q_OBJECT

public:
    View3DWindow(Controller *c, QWidget *parent = 0);
    ~View3DWindow();

	void add_mesh(ntk::Mesh &mesh);

protected:
    //Setup Methods
    void setup_connections();

public slots:
	//Actions
	//File
	void on_close();

	//CheckBoxes
    void on_saveMeshPushButton_clicked();
    void on_trianglePushButton_clicked();
    void on_surfelsPushButton_clicked();
    void on_pointCloudPushButton_clicked();
    void on_colorMappingCheckBox_toggled(bool checked);

	void on_capture_model();
	void on_capture_model_single_frame();
	void on_n_frames(int value);

	//Camera
    void on_resetCamera_clicked();

private:
    Ui::View3D *ui;
    Controller *_controller;

	ntk::MeshGenerator* _mesh_generator;

	ntk::Mesh *_mesh_temp;

	int _n_frames;
};

#endif // VIEW3DWINDOW_H