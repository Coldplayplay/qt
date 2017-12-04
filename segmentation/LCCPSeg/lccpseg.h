#ifndef LCCPSEG_H
#define LCCPSEG_H

// Stdlib
#include <stdlib.h>
#include <cmath>
#include <limits.h>

#include <boost/format.hpp>

// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

// VTK
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>
#include <vtkRenderWindow.h>

#include <QFileDialog>
#include <QMainWindow>


typedef pcl::PointXYZRGBA PointT;  // The point type used for input
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerRGBField;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
using namespace std;

namespace Ui {
class LCCPSeg;
}

class LCCPSeg : public QMainWindow
{
    Q_OBJECT

public:
    explicit LCCPSeg(QWidget *parent = 0);
    ~LCCPSeg();
    void initial();
    void viewPair();
    void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                      PointCloudT &adjacent_supervoxel_centers,
                                      std::string supervoxel_name,
                                      boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);


public Q_SLOTS:    

    void  loadButtonPressed();
    void  saveButtonPressed();
    void  lccpSeg();
    void  viewShow();
//paras
    void  spinBox1Changed(int value);
    void  spinBox2Changed(int value);
    void  spinBox3Changed(int value);
    void  spinBox4Changed(int value);
    void  spinBox5Changed(int value);
    void  spinBox6Changed(int value);
    void  spinBox7Changed(int value);
    void  spinBox8Changed(int value);
    void  checkBox_p1Changed(int value);
    void  checkBox_p2Changed(int value);
    void  checkBox_p3Changed(int value);
    void  checkBox_p4Changed(int value);
//viewer
    void  checkBox_s1Changed(int value);
    void  checkBox_s2Changed(int value);
    void  checkBox_s3Changed(int value);
    void  checkBox_s4Changed(int value);
    void  checkBox_s5Changed(int value);
    void  checkBox_show1Changed(int value);
    void  checkBox_show2Changed(int value);

private:
    Ui::LCCPSeg *ui;

//viewer Stuff
    bool show_supervoxels;
    bool show_lccp;
    bool show_voxel_centroids;
    bool show_supervoxel_normals;
    bool show_normals;
    bool show_graph;
    bool show_refined;

    bool refined_normal_shown;
    bool refined_sv_normal_shown;
    bool sv_added;
    bool normals_added;
    bool graph_added;
    std::vector<std::string> poly_names;
// Supervoxel Stuff
    float voxel_resolution;
    float seed_resolution;
    float color_importance;
    float spatial_importance;
    float normal_importance;
    bool use_single_cam_transform;
    bool use_supervoxel_refinement;

// LCCPSegmentation Stuff
    float concavity_tolerance_threshold;
    float smoothness_threshold;
    uint32_t min_segment_size;
    bool use_extended_convexity;
    bool use_sanity_criterion;
    unsigned int k_factor;
//data
    PointCloudT::Ptr cloudin;
    //PointCloudT::Ptr cloudout;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    int v1,v2;

    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
    PointLCloudT::Ptr labeled_voxel_cloud;
    PointCloudT::Ptr voxel_centroid_cloud;
    PointNCloudT::Ptr sv_normal_cloud;
    PointLCloudT::Ptr full_labeled_cloud;
    std::multimap<uint32_t, uint32_t> label_adjacency;

    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr> refined_supervoxel_clusters;
    PointLCloudT::Ptr refined_labeled_voxel_cloud;
    PointNCloudT::Ptr sv_centroid_normal_cloud;
    PointNCloudT::Ptr refined_sv_normal_cloud;
    PointLCloudT::Ptr refined_full_labeled_cloud;

    PointLCloudT::Ptr lccp_labeled_cloud;


    };

    #endif // LCCPSEG_H
