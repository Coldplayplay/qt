#ifndef SEMSEG_H
#define SEMSEG_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <Eigen/Core>
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
#include <pcl/common/pca.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
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

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGBL PointLT;
typedef pcl::PointCloud<PointLT> PointCloudLT;

using namespace std;
using namespace cv;
using namespace boost::python;
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
    void viewPair();

protected:
    int v1, v2;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud1;//输入
    PointCloudT::Ptr cloud3;//显示    
    PointCloudLT::Ptr cloud2;//用于位姿估计
    PointCloudT::Ptr cloud_tmp;

    string xyz1,xyz2,xyz3;
    string rpy1,rpy2,rpy3;

    bool show_cube;


public Q_SLOTS:
    void loadButtonPressed();
    void semsegButtonPressed();
    void showButtonPressed();
    void checkBox_cubeChanged(int value);
    void poseButtonPressed();
    void sendButtonPressed();

    //void xyzlineEditValueChanged();
    //void rpylineEditValueChanged();
    //void iplineEditValueChanged();

    void idSpinBoxValueChanged(QString value);
    void ipLineEditValueChanged(QString value);

private:
    Ui::SemSeg *ui;
    QString filename;
    string toBeSegFileName;
    string segedFileName;
    vector<Eigen::Quaternionf> xuan;
    vector<Eigen::Vector3f> ping;

    void split(const string& src, const string& delim, vector<string>& dest);
    void obj2pcd();
    void pcd2txt();
};

#endif // SEMSEG_H
