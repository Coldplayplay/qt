#include "lccpseg.h"
#include "../build/ui_lccpseg.h"

LCCPSeg::LCCPSeg(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LCCPSeg)
{
    ui->setupUi(this);
    this->setWindowTitle ("LCCPSegmentation viewer");

    initial();

//all paras
    connect(ui->spinBox_1,  SIGNAL(valueChanged(int)), this, SLOT(spinBox1Changed(int)));
    connect(ui->spinBox_2,  SIGNAL(valueChanged(int)), this, SLOT(spinBox2Changed(int)));
    connect(ui->spinBox_3,  SIGNAL(valueChanged(int)), this, SLOT(spinBox3Changed(int)));
    connect(ui->spinBox_4,  SIGNAL(valueChanged(int)), this, SLOT(spinBox4Changed(int)));
    connect(ui->spinBox_5,  SIGNAL(valueChanged(int)), this, SLOT(spinBox5Changed(int)));
    connect(ui->spinBox_6,  SIGNAL(valueChanged(int)), this, SLOT(spinBox6Changed(int)));
    connect(ui->spinBox_7,  SIGNAL(valueChanged(int)), this, SLOT(spinBox7Changed(int)));
    connect(ui->spinBox_8,  SIGNAL(valueChanged(int)), this, SLOT(spinBox8Changed(int)));
    connect (ui->checkBox_p1, SIGNAL (stateChanged(int)), this, SLOT (checkBox_p1Changed(int)));
    connect (ui->checkBox_p2, SIGNAL (stateChanged(int)), this, SLOT (checkBox_p2Changed(int)));
    connect (ui->checkBox_p3, SIGNAL (stateChanged(int)), this, SLOT (checkBox_p3Changed(int)));
    connect (ui->checkBox_p4, SIGNAL (stateChanged(int)), this, SLOT (checkBox_p4Changed(int)));


//viewer
    connect (ui->checkBox_s1, SIGNAL (stateChanged(int)), this, SLOT (checkBox_s1Changed(int)));
    connect (ui->checkBox_s2, SIGNAL (stateChanged(int)), this, SLOT (checkBox_s2Changed(int)));
    connect (ui->checkBox_s3, SIGNAL (stateChanged(int)), this, SLOT (checkBox_s3Changed(int)));
    connect (ui->checkBox_s4, SIGNAL (stateChanged(int)), this, SLOT (checkBox_s4Changed(int)));
    connect (ui->checkBox_s5, SIGNAL (stateChanged(int)), this, SLOT (checkBox_s5Changed(int)));
    connect (ui->checkBox_show1, SIGNAL (stateChanged(int)), this, SLOT (checkBox_show1Changed(int)));
    connect (ui->checkBox_show2, SIGNAL (stateChanged(int)), this, SLOT (checkBox_show2Changed(int)));

    connect (ui->checkBox_s1, SIGNAL (released()), this, SLOT(viewShow()));
    connect (ui->checkBox_s2, SIGNAL (released()), this, SLOT(viewShow()));
    connect (ui->checkBox_s3, SIGNAL (released()), this, SLOT(viewShow()));
    connect (ui->checkBox_s4, SIGNAL (released()), this, SLOT(viewShow()));
    connect (ui->checkBox_s5, SIGNAL (released()), this, SLOT(viewShow()));   
    connect (ui->checkBox_show1, SIGNAL (released()), this, SLOT(viewShow()));
    connect (ui->checkBox_show2, SIGNAL (released()), this, SLOT(viewShow()));

//key pushbutton
    connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));
    connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (loadButtonPressed()));
    connect (ui->pushButton_lccp,  SIGNAL (clicked ()), this, SLOT (lccpSeg()));

}

void LCCPSeg::initial()
{
//viewer Stuff
    show_supervoxels = false;
    show_lccp = true;

    show_voxel_centroids = false;
    show_supervoxel_normals = false;
    show_normals = false;
    show_graph = true;
    show_refined = true;

    refined_normal_shown = show_refined;
    refined_sv_normal_shown = show_refined;
    sv_added = false;
    normals_added = false;
    graph_added = false;

// Supervoxel Stuff
    voxel_resolution = 0.008f;//   -v 0.0075f
    seed_resolution = 0.08f;//   	-s 0.03f
    color_importance =0.2f;//	  -c 0.0f
    spatial_importance = 0.4f;//	-z 1.0f
    normal_importance = 1.0f;//	-n 4.0f
    use_single_cam_transform = false;
    use_supervoxel_refinement = false;

// LCCPSegmentation Stuff
    concavity_tolerance_threshold = 10;//	-ct
    smoothness_threshold = 0.1;//		-st
    min_segment_size = 0;//		-smooth
    use_extended_convexity = false;//	-ec
    use_sanity_criterion = false;//		-sc

    k_factor = 0;
    v1=0;v2=0;
    cloudin.reset (new PointCloudT); // Setup the cloud pointer
    //cloudout.reset (new PointCloudT);
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));

    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->initCameraParameters();

    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Before segmentation", 10, 10, "v1 text", v1);
    viewer->addPointCloud(cloudin, "v1", v1);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("After segmentation", 10, 10, "v2 text", v2);
    //viewer->addPointCloud(cloudin, "v2", v2);
    ui->qvtkWidget->update ();
}

void LCCPSeg::viewPair()
{
    viewer->removePointCloud("v1");
    //viewer->removePointCloud("v2");

    PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);
    //PointCloudColorHandlerRGBField<PointT> rgb2(cloudout);
    viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);
    //viewer->addPointCloud<PointT>(cloudout, rgb2, "v2", v2);

    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}
void LCCPSeg::addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
                                  pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

    //Iterate through all adjacent points, and add a center point to adjacent point pair
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
    for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
    {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    // Add the points to the dataset
    polyData->SetPoints (points);
    polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
    for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
    cells->InsertNextCell (polyLine);
    // Add the lines to the dataset
    polyData->SetLines (cells);
    viewer->addModelFromPolyData (polyData,supervoxel_name, v2);
}


void LCCPSeg::viewShow()
{
    cout<<"===================================================="<<endl;
    cout<<"1.show_supervoxels:"<<show_supervoxels<<endl;
    cout<<"2.show_lccp:"<<show_lccp<<endl;
    cout<<"3.show_voxel_centroids:"<<show_voxel_centroids<<endl;
    cout<<"4.show_supervoxel_normals:"<<show_supervoxel_normals<<endl;
    cout<<"5.show_normals:"<<show_normals<<endl;
    cout<<"6.show_graph:"<<show_graph<<endl;
    cout<<"7.show_refined:"<<show_refined<<endl;
    if (show_lccp)
    {
        cout<<"show lccp segmentation"<<endl;
        if (!viewer->updatePointCloud (lccp_labeled_cloud, "colored lccp"))
        {
          viewer->addPointCloud (lccp_labeled_cloud, "colored lccp", v2);
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3.0, "colored lccp");
        }
    }
    else
    {
      viewer->removePointCloud ("colored lccp");

    }


    if (show_supervoxels)
    {
      cout<<"show supervoxels segmentation"<<endl;
        if (!viewer->updatePointCloud ((show_refined)?refined_labeled_voxel_cloud:labeled_voxel_cloud, "colored voxels"))
      {
        viewer->addPointCloud ((show_refined)?refined_labeled_voxel_cloud:labeled_voxel_cloud, "colored voxels", v2);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3.0, "colored voxels");
      }
    }
    else
    {
      viewer->removePointCloud ("colored voxels");

    }

    if (show_voxel_centroids)
    {
      if (!viewer->updatePointCloud (voxel_centroid_cloud, "voxel centroids"))
      {
        viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids", v2);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
      }
    }
    else
    {
      viewer->removePointCloud ("voxel centroids");
    }

    if (show_supervoxel_normals)
    {
      if (refined_sv_normal_shown != show_refined || !sv_added)
      {
        viewer->removePointCloud ("supervoxel_normals");
        viewer->addPointCloudNormals<PointNT> ((show_refined)?refined_sv_normal_cloud:sv_normal_cloud,1,0.05f, "supervoxel_normals", v2);
        sv_added = true;
      }
      refined_sv_normal_shown = show_refined;
    }
    else if (!show_supervoxel_normals)
    {
      viewer->removePointCloud ("supervoxel_normals");
    }

    if (show_normals)
    {
      std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator sv_itr,sv_itr_end;
      sv_itr = ((show_refined)?refined_supervoxel_clusters.begin ():supervoxel_clusters.begin ());
      sv_itr_end = ((show_refined)?refined_supervoxel_clusters.end ():supervoxel_clusters.end ());
      for (; sv_itr != sv_itr_end; ++sv_itr)
      {
        std::stringstream ss;
        ss << sv_itr->first <<"_normal";
        if (refined_normal_shown != show_refined || !normals_added)
        {
          viewer->removePointCloud (ss.str ());
          viewer->addPointCloudNormals<PointT,NormalT> ((sv_itr->second)->voxels_,(sv_itr->second)->normals_,10,0.02f,ss.str (), v2);
        //  std::cout << (sv_itr->second)->normals_->points[0]<<"\n";

        }

      }
      normals_added = true;
      refined_normal_shown = show_refined;
    }
    else if (!show_normals)
    {
      std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator sv_itr,sv_itr_end;
      sv_itr = ((show_refined)?refined_supervoxel_clusters.begin ():supervoxel_clusters.begin ());
      sv_itr_end = ((show_refined)?refined_supervoxel_clusters.end ():supervoxel_clusters.end ());
      for (; sv_itr != sv_itr_end; ++sv_itr)
      {
        std::stringstream ss;
        ss << sv_itr->first << "_normal";
        viewer->removePointCloud (ss.str ());
      }
    }

    if (show_graph && !graph_added)
    {
      poly_names.clear ();
      std::multimap<uint32_t,uint32_t>::iterator label_itr = label_adjacency.begin ();
      for ( ; label_itr != label_adjacency.end (); )
      {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
         //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);
        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        //PointCloudT adjacent_supervoxel_centers;
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = label_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=label_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
        {
          pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
          adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        poly_names.push_back (ss.str ());
        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
        //Move iterator forward to next label
        label_itr = label_adjacency.upper_bound (supervoxel_label);
      }

      graph_added = true;
    }
    else if (!show_graph && graph_added)
    {
      for (std::vector<std::string>::iterator name_itr = poly_names.begin (); name_itr != poly_names.end (); ++name_itr)
      {
        viewer->removeShape (*name_itr);
      }
      graph_added = false;
    }
}

void LCCPSeg::loadButtonPressed()
{

   QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/cbc/图片/", tr ("Point cloud data (*.pcd *.ply)"));

   PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
   PointCloudT::Ptr cloud_tmp (new PointCloudT);

   if (filename.isEmpty ())
     return;

   int return_status;
   if (filename.endsWith (".pcd", Qt::CaseInsensitive))
     return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
   else
     return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

   if (return_status != 0)
   {
     PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
     return;
    }
   // If point cloud contains NaN values, remove them before updating the visualizer point cloud
   if (cloud_tmp->is_dense)
     pcl::copyPointCloud (*cloud_tmp, *cloudin);
   else
    {
     PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
     std::vector<int> vec;
     pcl::removeNaNFromPointCloud (*cloud_tmp, *cloudin, vec);
    }
   //pcl::copyPointCloud(*cloud_tmp,*cloudout);
   viewPair();

}

void LCCPSeg::saveButtonPressed()
{
    QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/cbc/图片/", tr ("Point cloud data (*.pcd *.ply)"));

      PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

      if (filename.isEmpty ())
        return;

      int return_status;
      if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::savePCDFile (filename.toStdString (), *lccp_labeled_cloud);
      else if (filename.endsWith (".ply", Qt::CaseInsensitive))
        return_status = pcl::io::savePLYFile (filename.toStdString (), *lccp_labeled_cloud);
      else
      {
        filename.append(".pcd");
        return_status = pcl::io::savePCDFile (filename.toStdString (), *lccp_labeled_cloud);
      }

      if (return_status != 0)
      {
        PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
        return;
      }
}

void LCCPSeg::lccpSeg()
{
    /// Preparation of Input: Supervoxel Oversegmentation
    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform (use_single_cam_transform);
    super.setInputCloud (cloudin);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);
    //std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;//mistake due to this line

    PCL_INFO ("Extracting supervoxels\n");
    super.extract (supervoxel_clusters);    

    labeled_voxel_cloud.reset(new PointLCloudT);
    voxel_centroid_cloud.reset(new PointCloudT);
    sv_normal_cloud.reset(new PointNCloudT);
    full_labeled_cloud.reset(new PointLCloudT); 

    labeled_voxel_cloud = super.getLabeledVoxelCloud ();
    voxel_centroid_cloud = super.getVoxelCentroidCloud ();
    sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
    full_labeled_cloud = super.getLabeledCloud ();

    std::stringstream temp;
    temp << "  Nr. Supervoxels: " << supervoxel_clusters.size () << "\n";
    PCL_INFO (temp.str ().c_str ());
    PCL_INFO ("Getting supervoxel adjacency\n");
    //std::multimap<uint32_t, uint32_t> label_adjacency;
    super.getSupervoxelAdjacency (label_adjacency);

    //std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> refined_supervoxel_clusters;//mistake due to this line
    refined_labeled_voxel_cloud.reset(new PointLCloudT);
    refined_sv_normal_cloud.reset(new PointNCloudT);
    refined_full_labeled_cloud.reset(new PointLCloudT);
    if (use_supervoxel_refinement)
    {
    PCL_INFO ("Refining supervoxels\n");
    super.refineSupervoxels (3, refined_supervoxel_clusters);
    refined_labeled_voxel_cloud = super.getLabeledVoxelCloud ();
    refined_sv_normal_cloud = super.makeSupervoxelNormalCloud (refined_supervoxel_clusters);
    refined_full_labeled_cloud = super.getLabeledCloud ();
    }

    /// The Main Step: Perform LCCPSegmentation

    PCL_INFO ("Starting Segmentation\n");
    pcl::LCCPSegmentation<PointT> lccp;
    lccp.setConcavityToleranceThreshold (concavity_tolerance_threshold);
    lccp.setSanityCheck (use_sanity_criterion);
    lccp.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
    if(use_extended_convexity)
        k_factor = 1;
    else
        k_factor = 0;
    lccp.setKFactor (k_factor);
    lccp.setMinSegmentSize (min_segment_size);
    if(use_supervoxel_refinement)
        lccp.setInputSupervoxels (refined_supervoxel_clusters, label_adjacency);
    else
        lccp.setInputSupervoxels (supervoxel_clusters, label_adjacency);
    lccp.segment ();

    PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");   
    lccp_labeled_cloud.reset(new PointLCloudT);
    if(use_supervoxel_refinement)
        lccp_labeled_cloud = refined_full_labeled_cloud->makeShared ();//cannot understand
    else
        lccp_labeled_cloud = full_labeled_cloud->makeShared ();

    lccp.relabelCloud (*lccp_labeled_cloud);
    SuperVoxelAdjacencyList sv_adjacency_list;
    lccp.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization
    cout<<"End up LCCP Segmentation."<<endl;
    viewShow();

}
//viewer
void LCCPSeg::checkBox_s1Changed(int value)
{
    show_voxel_centroids = value;
}
void LCCPSeg::checkBox_s2Changed(int value)
{
    show_supervoxel_normals = value;
}
void LCCPSeg::checkBox_s3Changed(int value)
{
    show_normals = value;
}
void LCCPSeg::checkBox_s4Changed(int value)
{
    show_graph = value;
}
void LCCPSeg::checkBox_s5Changed(int value)
{
    show_refined = value;
}
void LCCPSeg::checkBox_show1Changed(int value)
{
    show_supervoxels = value;
}
void LCCPSeg::checkBox_show2Changed(int value)
{
    show_lccp = value;
}

//paras
void  LCCPSeg::spinBox1Changed(int value)
{
    voxel_resolution = value/10000.0;
    ui->label_1->setText(QString::number(voxel_resolution, 'f'));
}
void  LCCPSeg::spinBox2Changed(int value)
{
    seed_resolution = value/100.0;
    ui->label_2->setText(QString::number(seed_resolution, 'f', 3));
}
void  LCCPSeg::spinBox3Changed(int value)
{
    color_importance = value/10.0;
    ui->label_3->setText(QString::number(color_importance, 'f', 2));
}
void  LCCPSeg::spinBox4Changed(int value)
{
    spatial_importance = value/10.0;
    ui->label_4->setText(QString::number(spatial_importance, 'f', 2));
}
void  LCCPSeg::spinBox5Changed(int value)
{
    normal_importance = value/10.0;
    ui->label_5->setText(QString::number(normal_importance, 'f', 2));
}
void  LCCPSeg::spinBox6Changed(int value)
{
    concavity_tolerance_threshold = value/1.0;
    ui->label_6->setText(QString::number(concavity_tolerance_threshold, 'f', 1));
}
void  LCCPSeg::spinBox7Changed(int value)
{
    smoothness_threshold = value/10.0;
    ui->label_7->setText(QString::number(smoothness_threshold, 'f', 2));
}
void  LCCPSeg::spinBox8Changed(int value)
{
    min_segment_size = value;
    ui->label_8->setText(QString::number(min_segment_size));
}

void LCCPSeg::checkBox_p1Changed(int value)
{
    use_single_cam_transform = value;
}
void LCCPSeg::checkBox_p2Changed(int value)
{
    use_supervoxel_refinement = value;
}
void LCCPSeg::checkBox_p3Changed(int value)
{
    use_extended_convexity = value;
}
void LCCPSeg::checkBox_p4Changed(int value)
{
    use_sanity_criterion = value;
}



LCCPSeg::~LCCPSeg()
{
    delete ui;
}
