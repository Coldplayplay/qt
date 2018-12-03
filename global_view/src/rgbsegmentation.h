#ifndef RGBSEGMENTATION_H
#define RGBSEGMENTATION_H

#include <iostream>
#include <string>
#include <QMainWindow>
#include <QFileDialog>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>


using namespace std;
using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerRGBField;

namespace Ui {
class RGBSegmentation;
}

class RGBSegmentation : public QMainWindow
{
    Q_OBJECT

public:
    explicit RGBSegmentation(QWidget *parent = 0);
    ~RGBSegmentation();
    void initial();
    void viewPair();    
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    void pcd2txt(string name, PointCloudT::Ptr cloud);

public Q_SLOTS:
    void  loadButtonPressed();
    void  saveButtonPressed();
    void  savesinglesButtonPressed();
    void  savesinglesButtonPressed1();
   
    void  checkBox_cubeChanged(int);
    void  showCube();
    
    void  spinBox1Changed(int value);
    void  spinBox2Changed(int value);
    void  spinBox3Changed(int value);
    void  spinBox4Changed(int value);
    void  segmentation();
    //static filter
    void  updateLabelValue(int value);
    void  valueChanged1(int value);
    void  valueChanged2(int value);   
    void  statisticFilter();

    //rotate and translation
    void  rSliderValueChanged (int value);
    void  pSliderValueChanged (int value);
    void  ySliderValueChanged (int value);
    void  rotate_UI();

    void  spinBox_numChanged(int value);
    
protected: 
    PointCloudT::Ptr cloudin;
    PointCloudT::Ptr temp;
    PointCloudT::Ptr cloudout;
    vector <pcl::PointIndices> clusters;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    int pre_clusters_num;   
    int v1,v2;
    int flag;

    bool show_cube; 
    
    //PointT min; 
    //PointT max;

    int DistanceThreshold;
    int PointColorThreshold;
    int RegionColorThreshold;
    int MinClusterSize; 

    QString filenameload;   
    //statis filter
    int MeanK;
    double StddevMulThresh;

    double roll,pitch,yaw;//RPY变量
    double x_trans,y_trans,z_trans;//平移变量
    int num_dir;
    
private:
    Ui::RGBSegmentation *ui;
    
};

#endif // RGBSEGMENTATION_H
