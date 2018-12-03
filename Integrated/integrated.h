#ifndef INTEGRATED_H
#define INTEGRATED_H

// C++
#include <iostream>
#include <string>
// Qt
#include <QMainWindow>
#include <QFileDialog>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerRGBField;

namespace Ui {
class Integrated;
}

class Integrated : public QMainWindow
{
    Q_OBJECT

public:
    explicit Integrated(QWidget *parent = 0);
    ~Integrated();
    void viewPair();
    void initial();
    Eigen::Matrix3d euler2RotationMatrix(double roll, double pitch, double yaw);

public Q_SLOTS:
    /*条件滤波*/   
    void  x1ValueChanged (double value);
    void  x2ValueChanged (double value);
    void  y1ValueChanged (double value);
    void  y2ValueChanged (double value);
    void  z1ValueChanged (double value);
    void  z2ValueChanged (double value);

    void  x1SliderValueChanged (int value);
    void  x2SliderValueChanged (int value);
    void  y1SliderValueChanged (int value);
    void  y2SliderValueChanged (int value);
    void  z1SliderValueChanged (int value);
    void  z2SliderValueChanged (int value);

    void  conditFilter();

    /*降采样*/
    void doublevalueChanged1(double value);
    void doublevalueChanged2(double value);
    void doublevalueChanged3(double value);
    void exectButtonPressed();

    /*离群点去除*/   
    void doublevalueChanged_mean(double value);
    void doublevalueChanged_std(double value);
    void statisticFilter();

    /*旋转*/
    void  rSliderValueChanged (int value);
    void  pSliderValueChanged (int value);
    void  ySliderValueChanged (int value);   
    void  rotate_UI();

    /*公共的*/
    void  saveButtonPressed ();
    void  loadButtonPressed ();
    void  resetButtonPressed();
    void  cancelButtonPressed();

protected:
    int v1, v2, v3, v4;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloudin;
    PointCloudT::Ptr cloudout;
    PointCloudT::Ptr cloudout1;
    PointCloudT::Ptr cloudout2;
    PointCloudT::Ptr cloudout3;
    PointCloudT::Ptr temp;

    bool rotate_flag;
    double x1,x2,y1,y2,z1,z2;//conditional filter params
    double roll,pitch,yaw;//RPY变量
    double lx,ly,lz;
    int MeanK; double StddevMulThresh;


private:
    Ui::Integrated *ui;

};

#endif // INTEGRATED_H
