#include "rgbsegmentation.h"
#include "../build/ui_rgbsegmentation.h"

RGBSegmentation::RGBSegmentation(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::RGBSegmentation)
{
    ui->setupUi(this);
    this->setWindowTitle ("RGBSegmentation viewer");
    initial();
    connect(ui->pushButton_load, SIGNAL(clicked()), this, SLOT(loadButtonPressed()));
    connect(ui->pushButton_save, SIGNAL(clicked()), this, SLOT(saveButtonPressed()));
    connect(ui->pushButton_savesingles, SIGNAL(clicked()), this, SLOT(savesinglesButtonPressed()));

    connect(ui->spinBox_1, SIGNAL(valueChanged(int)), this, SLOT(spinBox1Changed(int)));
    connect(ui->spinBox_2, SIGNAL(valueChanged(int)), this, SLOT(spinBox2Changed(int)));
    connect(ui->spinBox_3, SIGNAL(valueChanged(int)), this, SLOT(spinBox3Changed(int)));
    connect(ui->spinBox_4, SIGNAL(valueChanged(int)), this, SLOT(spinBox4Changed(int)));
 
    connect(ui->spinBox_1, SIGNAL(editingFinished()), this, SLOT(segmentation()));
    connect(ui->spinBox_2, SIGNAL(editingFinished()), this, SLOT(segmentation()));
    connect(ui->spinBox_3, SIGNAL(editingFinished()), this, SLOT(segmentation()));
    connect(ui->spinBox_4, SIGNAL(editingFinished()), this, SLOT(segmentation()));

    //connect(ui->pushButton_seg, SIGNAL(clicked()), this, SLOT(segmentation()));

}

void RGBSegmentation::initial()
{
    DistanceThreshold = 20;
    PointColorThreshold = 10;
    RegionColorThreshold = 25;
    MinClusterSize = 300;   
    
    flag = 0;
    v1=0;v2=0;
    cloudin.reset (new PointCloudT); // Setup the cloud pointer
    cloudout.reset (new PointCloudT);   
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
    viewer->addPointCloud(cloudin, "v2", v2);
    ui->qvtkWidget->update ();
}

void RGBSegmentation::viewPair()
{
    viewer->removePointCloud("v1");
    viewer->removePointCloud("v2");
    flag = 0;
    for(int i=0; i<pre_clusters_num; i++)
    {
        stringstream ss;
        ss<<"cube"<<i;
        viewer->removeShape(ss.str());
    }   
    PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);
    PointCloudColorHandlerRGBField<PointT> rgb2(cloudout);
    viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);	
    viewer->addPointCloud<PointT>(cloudout, rgb2, "v2", v2);

    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}

void RGBSegmentation::loadButtonPressed()
{

   filenameload = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

   PCL_INFO("File chosen: %s\n", filenameload.toStdString ().c_str ());
   PointCloudT::Ptr cloud_tmp (new PointCloudT);

   if (filenameload.isEmpty ())
     return;

   int return_status;
   if (filenameload.endsWith (".pcd", Qt::CaseInsensitive))
     return_status = pcl::io::loadPCDFile (filenameload.toStdString (), *cloud_tmp);
   else
     return_status = pcl::io::loadPLYFile (filenameload.toStdString (), *cloud_tmp);

   if (return_status != 0)
   {
     PCL_ERROR("Error reading point cloud %s\n", filenameload.toStdString ().c_str ());
     return;
    }    
   
   if (cloud_tmp->is_dense)
     pcl::copyPointCloud (*cloud_tmp, *cloudin);
   else
    {
     PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
     std::vector<int> vec;
     pcl::removeNaNFromPointCloud (*cloud_tmp, *cloudin, vec);
    }  
   pcl::copyPointCloud(*cloudin,*cloudout); 
   viewPair();

}

void RGBSegmentation::saveButtonPressed()
{
    QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

      PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

      if (filename.isEmpty ())
        return;

      int return_status;
      if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloudout);
      else if (filename.endsWith (".ply", Qt::CaseInsensitive))
        return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloudout);
      else
      {
        filename.append(".pcd");
        return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloudout);
      }

      if (return_status != 0)
      {
        PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
        return;
      }
}

void  RGBSegmentation::savesinglesButtonPressed()
{
    PointCloudT::Ptr cloudsingle (new PointCloudT);
    pcl::PCDWriter writer;
    
    int i = 1;
    string ldname = filenameload.toUtf8().constData();//QString to String
    char* name = strchr(&ldname[0], '.');
    size_t namelen = name-&ldname[0];
    string loadname = ldname.substr(0,namelen);

    pcl::ExtractIndices<PointT> extract;    
    extract.setInputCloud (cloudin);
    extract.setNegative (false);
    //for(vector<pcl::PointIndices>::const_iterator iter = clusters.cbegin(); iter!=clusters.cend(); iter++)
    //for(auto iter = clusters.cbegin())
    for(auto val:clusters)
    {
        pcl::PointIndices::Ptr tem (new pcl::PointIndices(val));//transform pcl::PointIndices to pcl::PointIndices::Ptr
        extract.setIndices (tem);        
        extract.filter (*cloudsingle);
        std::stringstream ss;
        ss << loadname <<"." << i << ".pcd";
        cout<<"文件写入了"<<ss.str()<<endl;
        writer.write<PointT> (ss.str (), *cloudsingle);
        cout<<"完成了"<<ss.str()<<"写入"<<endl;
        i++; 
    }
    return;
    
}

void RGBSegmentation::segmentation()
{
    pcl::IndicesPtr indices (new vector <int>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloudin);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 4.0);
    pass.filter (*indices);   
  
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
   
    pcl::RegionGrowingRGB<PointT> reg;
    reg.setInputCloud (cloudin);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (DistanceThreshold);//参数是点的个数还是距离？  determine whether the point is neighbouring or not.
    reg.setPointColorThreshold (PointColorThreshold);//判断是不是一类，点云颜色之差的阈值
    reg.setRegionColorThreshold (RegionColorThreshold);//判断两类能不能合并的点云颜色之差的阈值
    reg.setMinClusterSize (MinClusterSize);
    
    reg.extract (clusters);
    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl; 
  
 
    PointCloudT::Ptr colored_cloud = reg.getColoredCloud ();  
    PointCloudT::Ptr cloudsingle (new PointCloudT);
    PointCloudT proj; 
    
    if(clusters.size()==0)
    {
        cout<<"Failure in rgb clustering.\nPlease try some other parameters."<<endl;
        return;
    }
   else
    {
        cout<<"Success in rgb clustering."<<endl;
        *cloudout = *colored_cloud;
        viewer->removePointCloud("v2");       
        PointCloudColorHandlerRGBField<PointT> rgb2(cloudout);  
        viewer->addPointCloud<PointT>(cloudout, rgb2, "v2", v2);
        ui->qvtkWidget->update();

        if(flag!=0)
        {
            cout<<"去除之前的包围盒..."<<endl;
            for(int i=0; i<pre_clusters_num; i++)
            {
                stringstream ss;
                ss<<"cube"<<i;
                viewer->removeShape(ss.str());
            }
            
            ui->qvtkWidget->update();
        }
        flag = 1;
        pre_clusters_num = clusters.size();

        pcl::ExtractIndices<PointT> extract;    
        extract.setInputCloud (cloudout);
        extract.setNegative (false);

        pcl::PCA<PointT> pca;
        
        double a[10] = {1.0, 0.0, 0.0, 0.2, 0.4, 0.6}; 
        double b[10] = {0.0, 1.0, 0.0, 0.2, 0.4, 0.6};
        double c[10] = {0.0, 0.0, 1.0, 0.2, 0.4, 0.6};    
        int i = 0; 
        for(auto val:clusters)
       {
        pcl::PointIndices::Ptr tem (new pcl::PointIndices(val));//transform pcl::PointIndices to pcl::PointIndices::Ptr
        extract.setIndices (tem);        
        extract.filter (*cloudsingle);
/*
        pca.setInputCloud(cloudsingle);
        pca.project(*cloudsingle,proj);
        PointT proj_min; 
        PointT proj_max; 
        pcl::getMinMax3D (proj, proj_min, proj_max); 
        pca.reconstruct (proj_min, min); 
        pca.reconstruct (proj_max, max);

        cout<<"min:"<<"("<<min.x<<", "<<min.y<<", "<<min.z<<")"<<endl;
        cout<<"max:"<<"("<<max.x<<", "<<max.y<<", "<<max.z<<")"<<endl;
        stringstream ss;
        ss<<"cube"<<i;        
        viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, a[i], b[i], c[i], ss.str(), v2);
        cout<<"当前id为："<<ss.str()<<endl;
        i++;
*/
        // compute principal direction
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloudsingle, centroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*cloudsingle, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
        eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

        // move the points to the that reference frame
        Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
        p2w.block<3,3>(0,0) = eigDx.transpose();
        p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
        pcl::PointCloud<PointT> cPoints;
        pcl::transformPointCloud(*cloudsingle, cPoints, p2w);

        PointT min_pt, max_pt;
        pcl::getMinMax3D(cPoints, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

        // final transform
        const Eigen::Quaternionf qfinal(eigDx);
        const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

        // draw the cloud and the box
        stringstream ss;
        ss<<"cube"<<i;      
        cout<<"当前id为："<<ss.str()<<endl;               
        viewer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z, ss.str(), v2);
        i++;
       }   
       ui->qvtkWidget->update();   

    }       

}


void RGBSegmentation::spinBox1Changed(int value)
{
    DistanceThreshold = value;
    cout<<"四个值为："<<DistanceThreshold<<" "<<PointColorThreshold<<" "<<RegionColorThreshold<<" "<<MinClusterSize<<endl;
}

void RGBSegmentation::spinBox2Changed(int value)
{
    PointColorThreshold = value;
    cout<<"四个值为："<<DistanceThreshold<<" "<<PointColorThreshold<<" "<<RegionColorThreshold<<" "<<MinClusterSize<<endl;
}

void RGBSegmentation::spinBox3Changed(int value)
{
    RegionColorThreshold = value;
    cout<<"四个值为："<<DistanceThreshold<<" "<<PointColorThreshold<<" "<<RegionColorThreshold<<" "<<MinClusterSize<<endl;
}

void RGBSegmentation::spinBox4Changed(int value)
{
    MinClusterSize = value;
    cout<<"四个值为："<<DistanceThreshold<<" "<<PointColorThreshold<<" "<<RegionColorThreshold<<" "<<MinClusterSize<<endl;
}

RGBSegmentation::~RGBSegmentation()
{
    delete ui;
}
