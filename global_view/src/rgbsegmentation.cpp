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
    connect(ui->pushButton_savesingles, SIGNAL(clicked()), this, SLOT(savesinglesButtonPressed1()));
    
    connect(ui->checkBox_cube, SIGNAL(stateChanged(int)), this, SLOT(checkBox_cubeChanged(int)));
    connect(ui->checkBox_cube, SIGNAL(released()), this, SLOT(showCube()));

    connect(ui->spinBox_1, SIGNAL(valueChanged(int)), this, SLOT(spinBox1Changed(int)));
    connect(ui->spinBox_2, SIGNAL(valueChanged(int)), this, SLOT(spinBox2Changed(int)));
    connect(ui->spinBox_3, SIGNAL(valueChanged(int)), this, SLOT(spinBox3Changed(int)));
    connect(ui->spinBox_4, SIGNAL(valueChanged(int)), this, SLOT(spinBox4Changed(int)));
 
    connect(ui->spinBox_1, SIGNAL(editingFinished()), this, SLOT(segmentation()));
    connect(ui->spinBox_2, SIGNAL(editingFinished()), this, SLOT(segmentation()));
    connect(ui->spinBox_3, SIGNAL(editingFinished()), this, SLOT(segmentation()));
    connect(ui->spinBox_4, SIGNAL(editingFinished()), this, SLOT(segmentation()));

    //connect(ui->pushButton_seg, SIGNAL(clicked()), this, SLOT(segmentation()));

    connect (ui->spinBox_6, SIGNAL (valueChanged (int)), this, SLOT (updateLabelValue(int)));
    connect (ui->spinBox_5, SIGNAL (valueChanged (int)), this, SLOT (valueChanged1(int)));
    connect (ui->spinBox_6, SIGNAL (valueChanged (int)), this, SLOT (valueChanged2(int)));
    connect (ui->spinBox_5, SIGNAL(editingFinished()), this, SLOT (statisticFilter()));
    connect (ui->spinBox_6, SIGNAL(editingFinished()), this, SLOT (statisticFilter()));

    //rotate and translation
    connect (ui->horizontalSlider_r, SIGNAL (valueChanged (int)), this, SLOT (rSliderValueChanged (int)));
    connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));
    connect (ui->horizontalSlider_y, SIGNAL (valueChanged (int)), this, SLOT (ySliderValueChanged (int)));
    connect (ui->horizontalSlider_r, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
    connect (ui->horizontalSlider_p, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
    connect (ui->horizontalSlider_y, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
    connect (ui->spinBox_r, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));
    connect (ui->spinBox_p, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));
    connect (ui->spinBox_y, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));
    connect (ui->spinBox_num, SIGNAL(valueChanged (int)), this, SLOT (spinBox_numChanged(int)));

}


void RGBSegmentation::initial()
{
    DistanceThreshold = 20;
    PointColorThreshold = 7;
    RegionColorThreshold = 10;
    MinClusterSize = 300;   

    MeanK = 80;
    StddevMulThresh = 1.0;

    roll = 0;  pitch = 0; yaw = 0;
    num_dir = 1;
    
    flag = 0;
    show_cube = false;
    pre_clusters_num = 0;
    v1=0;v2=0;
    temp.reset(new PointCloudT);
    cloudin.reset (new PointCloudT); // Setup the cloud pointer
    cloudout.reset (new PointCloudT);   
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));

    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->initCameraParameters();
    
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Before segmentation", 10, 10, "v1 text", v1);
    
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("After segmentation", 10, 10, "v2 text", v2);

    viewer->addCoordinateSystem (1.0, "axis", v1);

    viewPair();
}

void RGBSegmentation::viewPair()
{
    viewer->removePointCloud("v1");
    viewer->removePointCloud("v2");
    //flag = 0;

    if(flag&&show_cube)
    {
       for(int i=0; i<pre_clusters_num; i++)
        {
            stringstream ss;
            ss<<"cube"<<i;
            viewer->removeShape(ss.str());
/*
            stringstream ss1;
            ss1 << "text" << i;
            viewer->removeText(ss1.str());
*/            
        } 
    }

/*
    PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);
    PointCloudColorHandlerRGBField<PointT> rgb2(cloudout);
    viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);	
    viewer->addPointCloud<PointT>(cloudout, rgb2, "v2", v2);
*/
    viewer->addPointCloud<PointT>(cloudin, "v1", v1);	
    viewer->addPointCloud<PointT>(cloudout, "v2", v2);

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
   pcl::copyPointCloud(*cloudin, *cloudout); 
   pcl::copyPointCloud(*cloudin, *temp); 
   viewPair();
   flag = 0;

}

void RGBSegmentation::saveButtonPressed()
{
    QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

      PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

      if (filename.isEmpty ())
        return;

      int return_status;
      if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::savePCDFile(filename.toStdString (), *cloudout);
      else if (filename.endsWith (".ply", Qt::CaseInsensitive))
        return_status = pcl::io::savePLYFile(filename.toStdString (), *cloudout);
      else
      {
        filename.append(".pcd");
        return_status = pcl::io::savePCDFile(filename.toStdString (), *cloudout);
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
    std::cerr << "PointCloud : " << cloudin->width * cloudin->height 
       << " data points (" << pcl::getFieldsList (*cloudin) << ").";
    extract.setNegative (false);
    //for(vector<pcl::PointIndices>::const_iterator iter = clusters.cbegin(); iter!=clusters.cend(); iter++)
    //for(auto iter = clusters.cbegin())
    for(auto val:clusters)
    {
        pcl::PointIndices::Ptr tem (new pcl::PointIndices(val));//transform pcl::PointIndices to pcl::PointIndices::Ptr
        extract.setIndices (tem);

        pcl::PointIndices tem1 = *tem;

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


void RGBSegmentation::pcd2txt(string name, PointCloudT::Ptr cloud)
{
  ofstream out(name);
  if(out.is_open())
  {
    for(int i=0; i<cloud->size(); i++)
    {    
      float x = cloud->points[i].x;
      float y = cloud->points[i].y;
      float z = cloud->points[i].z;      
      int r = cloud->points[i].r;
      int g = cloud->points[i].g;
      int b = cloud->points[i].b;//困惑是为什么不能直接用cout输出，非给强行转换成int类型才行

      out<<x<<" ";
      out<<y<<" ";
      out<<z<<" ";
      out<<r<<" ";
      out<<g<<" ";
      out<<b<<" ";
      
      out<<"\n";

    }   
  }

   out.close();

}

//保存到pointnet/data下面
void  RGBSegmentation::savesinglesButtonPressed1()
{
    PointCloudT::Ptr cloudsingle (new PointCloudT);
    pcl::PCDWriter writer;
    
    int i = 1;
    // string ldname = filenameload.toUtf8().constData();//QString to String
    // char* name = strchr(&ldname[0], '.');
    // size_t namelen = name-&ldname[0];
    // string j = ldname.substr(namelen-1, 1);//不够智能
    // int j_num = atoi(j.c_str());
    string loadname = "/home/cbc/DL/PointNet/pointnet/data/5081Dataset/";

    pcl::ExtractIndices<PointT> extract;    
    extract.setInputCloud (cloudin);
    std::cerr << "PointCloud : " << cloudin->width * cloudin->height 
       << " data points (" << pcl::getFieldsList (*cloudin) << ")."<<endl;
    extract.setNegative (false);
    //for(vector<pcl::PointIndices>::const_iterator iter = clusters.cbegin(); iter!=clusters.cend(); iter++)
    //for(auto iter = clusters.cbegin())
    for(auto val:clusters)
    {
        pcl::PointIndices::Ptr tem (new pcl::PointIndices(val));//transform pcl::PointIndices to pcl::PointIndices::Ptr
        extract.setIndices (tem);

        pcl::PointIndices tem1 = *tem;

        extract.filter (*cloudsingle);
        std::stringstream ss;
        ss << loadname <<"scene_" << num_dir << "/Annotations/arrest_"<<i<<".pcd";
        cout<<"文件写入了"<<ss.str()<<endl;
        writer.write<PointT> (ss.str (), *cloudsingle);
        //cout<<"完成了"<<ss.str()<<"写入"<<endl;

        std::stringstream ss1;
        ss1 << loadname <<"scene_" << num_dir << "/Annotations/arrest_"<<i<<".txt";
        cout<<"文件写入了"<<ss1.str()<<endl;
        pcd2txt(ss1.str(), cloudsingle);
        i++; 
    }
    return;
    
}

void RGBSegmentation::checkBox_cubeChanged(int value)
{
    show_cube = value;
}

void RGBSegmentation::showCube()
{
    if(flag&&show_cube)
    {
        pcl::ExtractIndices<PointT> extract;    
        extract.setInputCloud (cloudout);
        extract.setNegative (false);

        PointCloudT::Ptr cloudsingle (new PointCloudT);
        PointCloudT proj;

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

        /********************计算最小包围盒方法一*********************************/
        /*
        pca.setInputCloud(cloudsingle);
        pca.project(*cloudsingle,proj);
        PointT proj_min; 
        PointT proj_max;
        PointT min;
        PointT max; 
        pcl::getMinMax3D (proj, proj_min, proj_max); 
        pca.reconstruct (proj_min, min); 
        pca.reconstruct (proj_max, max);

        cout<<"min:"<<"("<<min.x<<", "<<min.y<<", "<<min.z<<")"<<endl;
        cout<<"max:"<<"("<<max.x<<", "<<max.y<<", "<<max.z<<")"<<endl;
        stringstream ss;
        ss<<"cube"<<i;        
        viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, a[i], b[i], c[i], ss.str(), v2);
        cout<<"当前id为："<<ss.str()<<endl;
        */

         /********************计算最小包围盒方法二*********************************/
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
        ss << "cube" << i;
        //stringstream ss1;
        //ss1 << "text" << i;

        viewer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z, ss.str(), v2);
        //viewer->addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z, a[i], b[i], c[i], ss.str(), v2);       
        //viewer->addText(ss.str(), 20, 20, a[i], b[i], c[i], ss1.str(), v2);
        
        cout << "当前id为：" << ss.str() << endl;       
        //cout << "旋转四元数：["<<qfinal.vec().transpose()<<"  "<<qfinal.w()<<"]"<<endl;
        cout << "旋转四元数：["<<qfinal.x()<<", "<<qfinal.y()<<", "<<qfinal.z()<<", "<<qfinal.w()<<"]"<<endl;
        //cout << "平移向量：["<<tfinal.transpose()<<"]"<<endl;
        cout << "平移向量：["<<tfinal[0]<<", "<<tfinal[1]<<", "<<tfinal[2]<<"]"<<endl;
        cout<<"min_pt:"<<"("<<min_pt.x<<", "<<min_pt.y<<", "<<min_pt.z<<")"<<endl;
        cout<<"max_pt:"<<"("<<max_pt.x<<", "<<max_pt.y<<", "<<max_pt.z<<")"<<endl;
        cout << endl;

        i++;
       }   
       ui->qvtkWidget->update();   
    }
    else
    {
        for(int i=0; i<pre_clusters_num; i++)
        {
            stringstream ss;
            ss<<"cube"<<i;            
            viewer->removeShape(ss.str());
/*
            stringstream ss1;
            ss1 << "text" << i;
            viewer->removeText3D(ss1.str());
*/
        }
    }
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
    std::cout << "------------------------------------------------------"<<std::endl;
    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;   
 
    PointCloudT::Ptr colored_cloud = reg.getColoredCloud ();  
    
    //PointCloudT proj; 
    
    if(clusters.size()==0)
    {
        cout<<"Failure in rgb clustering.\nPlease try some other parameters."<<endl;
        return;
    }
   else
    {
        cout<<"Success in rgb clustering."<<endl;
        flag = 1;
        *cloudout = *colored_cloud;

        viewPair();

        pre_clusters_num = clusters.size();

        showCube();

    }       

}

void RGBSegmentation::statisticFilter()
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    std::cout<<"离群点去除前，点云数目："<<temp->points.size()<<endl;
    sor.setInputCloud (temp);
    sor.setMeanK (MeanK);
    sor.setStddevMulThresh (StddevMulThresh);
    sor.filter (*cloudin);    
    std::cout<<"离群点去除后，点云数目："<<cloudin->points.size()<<endl;    

    viewer->removePointCloud("v1");
    PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);    
    viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);    
    ui->qvtkWidget->update ();

}

void  RGBSegmentation::rotate_UI()
{
  //Eigen::Matrix3d rotation_cal = euler2RotationMatrix(roll, pitch, yaw);
  //Eigen::Matrix4f trans = Eigen::Matrix4f::Identity(); 
  Eigen::Affine3f trans = Eigen::Affine3f::Identity();
  trans.rotate(Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
  trans.rotate(Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
  trans.rotate(Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ())); 
  //trans.translation() << 2.5, 0.0, 0.0;
  
  pcl::transformPointCloud(*cloudin, *cloudin, trans);
  std::cout<<trans.matrix()<<endl;
  viewer->removePointCloud("v1");
  PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);  
  viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);
  ui->qvtkWidget->update ();

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


void  RGBSegmentation::spinBox_numChanged(int value)
{
    num_dir = value;
    cout<<"当前选择存放目录为: "<<num_dir<<endl;
}

//statis filter
void RGBSegmentation::valueChanged1(int value)
{
    MeanK = value;
}

void RGBSegmentation::valueChanged2(int value)
{
    StddevMulThresh = value/100.0;
}

void RGBSegmentation::updateLabelValue(int value)
{
    double doublevalue = value/100.0;
    ui->label_num->setText(QString::number(doublevalue, 'f', 2));
}

//rotate and translation
void  RGBSegmentation::rSliderValueChanged (int value)
{
  roll = value/180.0*M_PI;
}
void  RGBSegmentation::pSliderValueChanged (int value)
{
  pitch = value/180.0*M_PI;
}
void  RGBSegmentation::ySliderValueChanged (int value)
{
  yaw = value/180.0*M_PI;
}

RGBSegmentation::~RGBSegmentation()
{
    delete ui;
}
