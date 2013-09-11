#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/segmentation/sac_segmentation.h>

//OUTLIERS
#include <pcl/filters/statistical_outlier_removal.h>

//SMOOTHING
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

//TRIANGULATION
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//VOXEL
#include <pcl/filters/voxel_grid.h>


#include "Main.h"


int main (int argc, char** argv){
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
//	// Load bun0.pcd -- should be available with the PCL archive in test 
//	
//	pcl::io::loadPLYFile ("floor_1_mirror.ply", *cloud);
//	pcl::io::loadPCDFile ("table_scene_lms400_inliers.pcd", *cloud2);

	//if (pcl::io::loadPLYFile("floor_1_mirror.ply", *cloud) == -1){ //* load the file	
	//	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	//	return (-1);
	//}

	//std::cout << "Loaded "
	//		<< cloud->width * cloud->height
	//		<< " data points from test_pcd.pcd with the following fields: "
	//		<< std::endl;

 // 
	////... populate cloud_file
	//pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	
	//pcl::visualization::PCLVisualizer viewer("Simple cloud_file Viewer");
	//viewer.setBackgroundColor (0, 0, 0);
	//viewer.addPointCloud(cloud->makeShared());

	//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coiso");
	//viewer.addCoordinateSystem (1.0);
	//viewer.initCameraParameters ();

	//pcl::visualization::PCLVisualizer viewer2("Simple cloud_file Viewer");
	//viewer2.setBackgroundColor (0, 0, 0);
	//viewer2.addPointCloud(cloud2->makeShared());

	//viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coiso2");
	//viewer2.addCoordinateSystem (1.0);
	//viewer2.initCameraParameters ();
 //
	//viewer.showCloud(cloud->makeShared());
	
	
	
	//
	//while (!viewer.wasStopped() && !viewer2.wasStopped()) 
	//{
	//	viewer.spinOnce (100);
	//	viewer2.spinOnce (100);
	//}

	
	//main_smoothing(argc,argv);
	//main_fast_triangulation(argc,argv);


	printf(")");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged (new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPLYFile("floor_2_mirror_even_better.ply", *merged) == -1){ //* load the file	
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}

	std::cout << "Loaded "
			<< merged->width * merged->height
			<< " data points from test_pcd.pcd with the following fields: "
			<< std::endl;


	//pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
	//voxel_grid.setInputCloud(merged);
	//voxel_grid.setLeafSize(0.002, 0.002, 0.002);
	//voxel_grid.setDownsampleAllData(false);
	//voxel_grid.filter(*merged);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(*merged, *vertices);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
	ne.setRadiusSearch (0.01);
	ne.setInputCloud (merged);
	ne.compute (*vertices);

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>); 
	tree2->setInputCloud (vertices); 

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.25);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (200);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(true);

	// Get result
	gp3.setInputCloud (vertices);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	//// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor (50, 50, 155);

	viewer.addPolygonMesh(triangles,"coiso");

	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coiso");
	//viewer.addPointCloudNormals<pcl::PointXYZRGBNormal> (vertices, 2, 0.02, "normals");
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();

	while (!viewer.wasStopped() ) 
	{
		viewer.spinOnce (100);
	}

	//sr->setSearchMethod(tree);
	//sr->setInputCloud(vertices);
	//sr->reconstruct(surface_);

	//main_remove_outliers(argc,argv);
	//main_smoothing(argc,argv);
	//main_fast_triangulation(argc,argv);


	return (0);
}

int main_remove_outliers(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPLYFile("floor_2_mirror_better.ply", *cloud) == -1){ //* load the file	
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}

	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from test_pcd.pcd with the following fields: "
			<< std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (25);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;


	//sor.setNegative (true);
	//sor.filter (*cloud_filtered);
	//writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);
	n.setInputCloud (cloud_filtered);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.5);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
	gp3.setMinimumAngle(M_PI/4); // 10 degrees
	gp3.setMaximumAngle(M_PI/2); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor (0, 0, 0);

	viewer.addPolygonMesh(triangles,"coiso");

	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coiso");
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();

	pcl::visualization::PCLVisualizer viewer2("Simple cloud_file Viewer");
	viewer2.setBackgroundColor (0, 0, 0);
	viewer2.addPointCloud(cloud_filtered->makeShared());

	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coiso2");
	viewer2.addCoordinateSystem (1.0);
	viewer2.initCameraParameters ();
 //
	//viewer.showCloud(cloud->makeShared());
		
	while (!viewer.wasStopped() && !viewer2.wasStopped()) 
	{
		viewer.spinOnce (100);
		viewer2.spinOnce (100);
	}

	return 0;
}

int main_smoothing(int argc, char** argv){
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
	// Load bun0.pcd -- should be available with the PCL archive in test 
	
	pcl::io::loadPLYFile ("floor_2_mirror_better.ply", *cloud);
	pcl::io::loadPLYFile ("floor_2_mirror_better.ply", *cloud2);

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
	mls.setComputeNormals (true);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);
	
	// Reconstruct
	mls.process (mls_points);

	// Save output
	pcl::visualization::PCLVisualizer viewer("Simple cloud_file Viewer");
	viewer.setBackgroundColor (0, 0, 0);
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coiso");
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();

	pcl::visualization::PCLVisualizer viewer2("Simple cloud_file Viewer");
	viewer2.setBackgroundColor (0, 0, 0);
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coiso2");
	viewer2.addCoordinateSystem (1.0);
	viewer2.initCameraParameters ();
 
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mls(&mls_points);
	//pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);

	viewer.addPointCloud(cloud->makeShared());
	//viewer.addPointCloudNormals<pcl::PointNormal> (mls_points, 2, 0.02, "normals");
	//viewer.addPointCloudNormals(cloud->makeShared(),mls_points.makeShared());
	viewer2.addPointCloud(cloud2->makeShared());
	while (!viewer.wasStopped() && !viewer2.wasStopped()) 
	{
		viewer.spinOnce (100);
		viewer2.spinOnce (100);

	}

	return 0;
}

int main_fast_triangulation(int argc, char** argv){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 cloud_blob;
	pcl::io::loadPLYFile ("floor_2_mirror_better.ply", cloud_blob);
	pcl::fromROSMsg (cloud_blob, *cloud);
	//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();


	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor (0, 0, 0);

	viewer.addPolygonMesh(triangles,"coiso");

	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coiso");
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce (100);
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	// Finish
	return (0);
}


/*
void write(){
	FILE* fp = fopen("file.pcd","w+");

	if(!fp) return;

	fprintf(fp,"# .PCD v0.7 - Point cloud_file Data file format\n");
	fprintf(fp,"VERSION 0.7\n");
	fprintf(fp,"FIELDS x y z\n");
	fprintf(fp,"SIZE 4 4 4\n");
	fprintf(fp,"TYPE F F F\n");
	fprintf(fp,"COUNT 1 1 1\n");
	fprintf(fp,"WIDTH 640\n");
	fprintf(fp,"HEIGHT 480\n");
	fprintf(fp,"VIEWPOINT 1 0 1 0 1 0 0\n");
		
	fprintf(fp,"POINTS 307200\n");
	fprintf(fp,"DATA ascii\n");

	int ac = 0;
	fprintf(fp,"0.0 0.0 0.0\n");ac++;
	for(float i = 0.0f ; i < 1.0f ; i+=0.1f){
		for(float j = 0.0f ; j < 1.0f ; j+=0.1f){
			fprintf(fp,"%f 0.2 %f\n",-1.0f-i,-1.0f-j);ac++;
		}
	}
	fprintf(fp,"0.0 0.0 0.0\n");ac++;

	for(int i = ac ; i < 307200 ; i++){
		fprintf(fp,"-1.5 0.0 -1.5\n");
	}

	fclose(fp);
}
*/