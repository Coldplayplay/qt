#ifndef SEMSEG_H
#define SEMSEG_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
// Qt
#include <QMainWindow>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// python
#include <boost/python.hpp>
using namespace std;
using namespace boost::python;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerRGBField;



namespace Ui {
class SemSeg;
}

class SemSeg : public QMainWindow
{
    Q_OBJECT

public:
    explicit SemSeg(QWidget *parent = 0);
    ~SemSeg();

    void initial();
    void viewpair();

protected:
    int v1, v2;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud1;
    PointCloudT::Ptr cloud2;


public Q_SLOTS:
    void obj_open();
    void semsegButtonPressed();


private:
    Ui::SemSeg *ui;
    void split(const string& src, const string& delim, vector<string>& dest);
    void obj2pcd(string filename);
};

#endif // SEMSEG_H
