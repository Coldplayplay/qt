#ifndef VOXELFILTER_H
#define VOXELFILTER_H

#include <QMainWindow>
#include <QFileDialog>

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using pcl::visualization::PointCloudColorHandlerRGBField;

namespace Ui {
class VoxelFilter;
}

class VoxelFilter : public QMainWindow
{
    Q_OBJECT

public:
    explicit VoxelFilter(QWidget *parent = 0);
    ~VoxelFilter();

    void initial();
    void viewPair();

public Q_SLOTS:

  void valueChanged1(int value);
  void valueChanged2(int value);
  void valueChanged3(int value);

  void exectButtonPressed();
  void saveButtonPressed ();
  void loadButtonPressed ();



private:
    Ui::VoxelFilter *ui;
    double lx;
    double ly;
    double lz;

    PointCloudT::Ptr cloudin;
    PointCloudT::Ptr cloudout;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    int v1,v2;
};

#endif // VOXELFILTER_H
