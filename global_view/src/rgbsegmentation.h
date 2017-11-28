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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
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


public Q_SLOTS:
    void  loadButtonPressed();
    void  saveButtonPressed();
    void  savesinglesButtonPressed();
    
    void  spinBox1Changed(int value);
    void  spinBox2Changed(int value);
    void  spinBox3Changed(int value);
    void  spinBox4Changed(int value);
    void  segmentation();
    
protected: 
    PointCloudT::Ptr cloudin;
    PointCloudT::Ptr cloudout;
    vector <pcl::PointIndices> clusters;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    int pre_clusters_num;   
    int v1,v2;
    int flag; 
    
    PointT min; 
    PointT max;

    int DistanceThreshold;
    int PointColorThreshold;
    int RegionColorThreshold;
    int MinClusterSize; 

    QString filenameload;   
    
private:
    Ui::RGBSegmentation *ui;
    
};

#endif // RGBSEGMENTATION_H
