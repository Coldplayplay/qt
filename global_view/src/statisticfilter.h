#ifndef STATISTICFILTER_H
#define STATISTICFILTER_H

#include <QMainWindow>
#include <QFileDialog>

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using pcl::visualization::PointCloudColorHandlerRGBField;

namespace Ui {
class StatisticFilter;
}

class StatisticFilter : public QMainWindow
{
    Q_OBJECT

public:
    explicit StatisticFilter(QWidget *parent = 0);
    ~StatisticFilter();
    void initial();
    void viewPair();

public Q_SLOTS:

  void updateLabelValue(int value);
  void valueChanged1(int value);
  void valueChanged2(int value);
  void saveButtonPressed ();
  void loadButtonPressed ();
  void statisticFilter();


private:
    Ui::StatisticFilter *ui;
    int MeanK;
    double StddevMulThresh;

    PointCloudT::Ptr cloudin;
    PointCloudT::Ptr cloudout;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    int v1,v2;


};

#endif // STATISTICFILTER_H
