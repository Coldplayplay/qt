#include "integrated.h"
#include "build/ui_integrated.h"

Integrated::Integrated(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Integrated)
{
      ui->setupUi(this);
      this->setWindowTitle ("Point Cloud Preprocess");

      initial();
      /*1条件滤波*/
      connect (ui->doubleSpinBox_x1, SIGNAL (valueChanged (double)), this, SLOT (x1ValueChanged (double)));
      connect (ui->doubleSpinBox_x2, SIGNAL (valueChanged (double)), this, SLOT (x2ValueChanged (double)));
      connect (ui->doubleSpinBox_y1, SIGNAL (valueChanged (double)), this, SLOT (y1ValueChanged (double)));
      connect (ui->doubleSpinBox_y2, SIGNAL (valueChanged (double)), this, SLOT (y2ValueChanged (double)));
      connect (ui->doubleSpinBox_z1, SIGNAL (valueChanged (double)), this, SLOT (z1ValueChanged (double)));
      connect (ui->doubleSpinBox_z2, SIGNAL (valueChanged (double)), this, SLOT (z2ValueChanged (double)));

      connect (ui->horizontalSlider_x1, SIGNAL (valueChanged (int)), this, SLOT (x1SliderValueChanged (int)));
      connect (ui->horizontalSlider_x2, SIGNAL (valueChanged (int)), this, SLOT (x2SliderValueChanged (int)));
      connect (ui->horizontalSlider_y1, SIGNAL (valueChanged (int)), this, SLOT (y1SliderValueChanged (int)));
      connect (ui->horizontalSlider_y2, SIGNAL (valueChanged (int)), this, SLOT (y2SliderValueChanged (int)));
      connect (ui->horizontalSlider_z1, SIGNAL (valueChanged (int)), this, SLOT (z1SliderValueChanged (int)));
      connect (ui->horizontalSlider_z2, SIGNAL (valueChanged (int)), this, SLOT (z2SliderValueChanged (int)));

      connect (ui->horizontalSlider_x1, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
      connect (ui->horizontalSlider_x2, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
      connect (ui->horizontalSlider_y1, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
      connect (ui->horizontalSlider_y2, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
      connect (ui->horizontalSlider_z1, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
      connect (ui->horizontalSlider_z2, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));

      connect (ui->doubleSpinBox_x1, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
      connect (ui->doubleSpinBox_x2, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
      connect (ui->doubleSpinBox_y1, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
      connect (ui->doubleSpinBox_y2, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
      connect (ui->doubleSpinBox_z1, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
      connect (ui->doubleSpinBox_z2, SIGNAL(editingFinished()), this, SLOT (conditFilter()));


      /*2降采样*/
      connect (ui->doubleSpinBox_1, SIGNAL (valueChanged (double)), this, SLOT (doublevalueChanged1(double)));
      connect (ui->doubleSpinBox_2, SIGNAL (valueChanged (double)), this, SLOT (doublevalueChanged2(double)));
      connect (ui->doubleSpinBox_3, SIGNAL (valueChanged (double)), this, SLOT (doublevalueChanged3(double)));
      connect (ui->pushButton_exect, SIGNAL(clicked()), this, SLOT (exectButtonPressed()));

      /*3离群点*/     
      connect (ui->doubleSpinBox_mean, SIGNAL (valueChanged (double)), this, SLOT (doublevalueChanged_mean(double)));
      connect (ui->doubleSpinBox_std, SIGNAL (valueChanged (double)), this, SLOT (doublevalueChanged_std(double)));
      connect (ui->doubleSpinBox_mean, SIGNAL(editingFinished()), this, SLOT (statisticFilter()));
      connect (ui->doubleSpinBox_std, SIGNAL(editingFinished()), this, SLOT (statisticFilter()));

      /*4旋转*/
      connect (ui->horizontalSlider_r, SIGNAL (valueChanged (int)), this, SLOT (rSliderValueChanged (int)));
      connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));
      connect (ui->horizontalSlider_y, SIGNAL (valueChanged (int)), this, SLOT (ySliderValueChanged (int)));

      connect (ui->horizontalSlider_r, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
      connect (ui->horizontalSlider_p, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
      connect (ui->horizontalSlider_y, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
      connect (ui->spinBox_r, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));
      connect (ui->spinBox_p, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));
      connect (ui->spinBox_y, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));

      /*公共的*/
      connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));
      connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (loadButtonPressed()));
      connect (ui->pushButton_cancel,  SIGNAL (clicked ()), this, SLOT (cancelButtonPressed()));
      connect (ui->pushButton_reset,  SIGNAL (clicked ()), this, SLOT (resetButtonPressed()));
}

/*1条件滤波begin*/
void Integrated::initial()
{
      x1 = -2.0;  x2 = 2;  y1 = -2.0;  y2 = 2;  z1 = 0.0;  z2 = 3;  // The default value
      roll = 0;  pitch = 0; yaw = 0;
      lx = 1.0;   ly = 1.0;   lz = 1.0;
      MeanK = 50;  StddevMulThresh = 0.4;
      rotate_flag=false;
      cloudin.reset (new PointCloudT); // Setup the cloud pointer
      cloudout.reset (new PointCloudT);
      cloudout1.reset (new PointCloudT);
      cloudout2.reset (new PointCloudT);
      cloudout3.reset (new PointCloudT);
      temp.reset(new PointCloudT);
      initial_temp.reset(new PointCloudT);
      viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
      v1=0;v2=0;v3=0;v4=0;
      // Set up the QVTK window
      ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
      viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
      //ui->qvtkWidget->update ();
      viewer->initCameraParameters();

      viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
      viewer->setBackgroundColor(255, 255, 255, v1);
      viewer->addText("Raw", 10, 10, 20,0,0,0,"v1 text", v1);
      viewer->addPointCloud(cloudin, "v1", v1);

      viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
      viewer->setBackgroundColor(255, 255, 255, v2);
      viewer->addText("After cond", 10, 10, 20,0,0,0,"v2 text", v2);
      viewer->addPointCloud(cloudin, "v2", v2);

      viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
      viewer->setBackgroundColor(255, 255, 255, v3);
      viewer->addText("After statistic", 10, 10, 20,0,0,0,"v3 text", v3);
      viewer->addPointCloud(cloudin, "v3", v3);

      viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
      viewer->setBackgroundColor(255, 255, 255, v4);
      viewer->addText("After voxel", 10, 10, 20,0,0,0,"v4 text", v4);
      viewer->addPointCloud(cloudin, "v4", v4);


      viewer->addCoordinateSystem (2.5, "axis", v1);

      ui->qvtkWidget->update ();

}

void Integrated::viewPair()
{
  viewer->removePointCloud("v1");
  viewer->removePointCloud("v2");
  viewer->removePointCloud("v3");
  viewer->removePointCloud("v4");
  PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);
  PointCloudColorHandlerRGBField<PointT> rgb2(cloudin);
  PointCloudColorHandlerRGBField<PointT> rgb3(cloudin);
  PointCloudColorHandlerRGBField<PointT> rgb4(cloudin);
  viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);
  viewer->addPointCloud<PointT>(initial_temp, rgb2, "v2", v2);
  viewer->addPointCloud<PointT>(initial_temp, rgb3, "v3", v3);
  viewer->addPointCloud<PointT>(initial_temp, rgb4, "v4", v4);
  viewer->resetCamera ();
  ui->qvtkWidget->update ();

}


Eigen::Matrix3d Integrated::euler2RotationMatrix(double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle*pitchAngle*rollAngle;
    Eigen::Matrix3d r = q.matrix();
    return r;
}

void Integrated::conditFilter()
{
  pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT>());

  range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
      pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, x1)));
  range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
      pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, x2)));

  range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
      pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, y1)));
  range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
      pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, y2)));

  range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
      pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, z1)));
  range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
      pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, z2)));
  if(cloudin->points.size())
  {
    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition (range_cond);
    if(rotate_flag)
        condrem.setInputCloud (temp);
    else
        condrem.setInputCloud (cloudin);
    condrem.setKeepOrganized(true);
    condrem.filter (*cloudout1);

    if (cloudout1->is_dense)
      pcl::copyPointCloud (*cloudout1, *cloudout);
    else
    {
      PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
      std::vector<int> vec;
      pcl::removeNaNFromPointCloud (*cloudout1, *cloudout1, vec);
      pcl::copyPointCloud (*cloudout1, *cloudout);
    }


    viewer->removePointCloud("v2");
    PointCloudColorHandlerRGBField<PointT> rgb2(cloudout1);
    viewer->addPointCloud<PointT>(cloudout1, rgb2, "v2", v2);
    ui->qvtkWidget->update ();

    //std::cout<<"当前x,y,z范围分别为:"<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<" "<<z1<<" "<<z2<<std::endl;
    printf ("当前x,y,z范围分别为: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
    std::cout<<"感兴趣区域选择前，点云数目："<<cloudin->points.size()<<endl;
    std::cout<<"感兴趣区域选择后，点云数目："<<cloudout1->points.size()<<endl;
    //std::cout<<"has finished conditional filtering."<<std::endl;
  }

}


void Integrated::x1ValueChanged (double value)
{
  x1 = value;
  ui->horizontalSlider_x1->setValue((int)(x1*100));
  //printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void Integrated::x2ValueChanged (double value)
{
  x2 = value;
  ui->horizontalSlider_x2->setValue((int)(x2*100));
  //printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void Integrated::y1ValueChanged (double value)
{
  y1 = value;
  ui->horizontalSlider_y1->setValue((int)(y1*100));
  //printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void Integrated::y2ValueChanged (double value)
{
  y2 = value;
  ui->horizontalSlider_y2->setValue((int)(y2*100));
  //printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void Integrated::z1ValueChanged (double value)
{
  z1 = value;
  ui->horizontalSlider_z1->setValue((int)(z1*100));
  //printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void Integrated::z2ValueChanged (double value)
{
  z2 = value;
  ui->horizontalSlider_z2->setValue((int)(z2*100));
  //printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void Integrated::x1SliderValueChanged (int value)
{
  ui->doubleSpinBox_x1->setValue(double(value/100.0));
}

void Integrated::x2SliderValueChanged (int value)
{
  ui->doubleSpinBox_x2->setValue(double(value/100.0));
}

void Integrated::y1SliderValueChanged (int value)
{
  ui->doubleSpinBox_y1->setValue(double(value/100.0));

}

void Integrated::y2SliderValueChanged (int value)
{
  ui->doubleSpinBox_y2->setValue(double(value/100.0));
}

void Integrated::z1SliderValueChanged (int value)
{
  ui->doubleSpinBox_z1->setValue(double(value/100.0));
}

void Integrated::z2SliderValueChanged (int value)
{
  ui->doubleSpinBox_z2->setValue(double(value/100.0));
}

/*2离群点去除*/
void Integrated::statisticFilter()
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloudout1);
    sor.setMeanK (MeanK);
    sor.setStddevMulThresh (StddevMulThresh);
    sor.filter (*cloudout2);
    if (cloudout2->is_dense)
      pcl::copyPointCloud (*cloudout2, *cloudout);
    else
    {
      PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
      std::vector<int> vec;
      pcl::removeNaNFromPointCloud (*cloudout2, *cloudout2, vec);
      pcl::copyPointCloud (*cloudout2, *cloudout);
    }


    std::cout<<"离群点去除前，点云数目："<<cloudout1->points.size()<<endl;
    std::cout<<"离群点去除后，点云数目："<<cloudout2->points.size()<<endl;

    viewer->removePointCloud("v3");
    PointCloudColorHandlerRGBField<PointT> rgb2(cloudout2);
    viewer->addPointCloud<PointT>(cloudout2, rgb2, "v3", v3);
    ui->qvtkWidget->update ();

}

void Integrated::doublevalueChanged_mean(double value)
{
    MeanK = value;
}

void Integrated::doublevalueChanged_std(double value)
{
    StddevMulThresh = value;
}


/*3体素格降采样*/

void Integrated::exectButtonPressed()
{
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloudout2);
    sor.setLeafSize (lx, ly, lz);
    sor.filter (*cloudout3);
    if (cloudout3->is_dense)
      pcl::copyPointCloud (*cloudout3, *cloudout);
    else
    {
      PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
      std::vector<int> vec;
      pcl::removeNaNFromPointCloud (*cloudout3, *cloudout3, vec);
      pcl::copyPointCloud (*cloudout3, *cloudout);
    }

    std::cout<<"降采样前，点云数目："<<cloudout2->points.size()<<endl;
    std::cout<<"降采样后，点云数目："<<cloudout3->points.size()<<endl;

    viewer->removePointCloud("v4");
    PointCloudColorHandlerRGBField<PointT> rgb2(cloudout3);
    viewer->addPointCloud<PointT>(cloudout3, rgb2, "v4", v4);
    ui->qvtkWidget->update ();

}
void Integrated::doublevalueChanged1(double value)
{
    lx = value;
}

void Integrated::doublevalueChanged2(double value)
{
    ly = value;
}

void Integrated::doublevalueChanged3(double value)
{
    lz = value/1000.0;    
}



/*旋转*/
void  Integrated::rotate_UI()
{
  rotate_flag=true;
  //Eigen::Matrix3d rotation_cal = euler2RotationMatrix(roll, pitch, yaw);
  //Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  Eigen::Affine3f trans = Eigen::Affine3f::Identity();
  trans.rotate(Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
  trans.rotate(Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
  trans.rotate(Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*cloudin, *temp, trans);
  pcl::copyPointCloud (*temp, *cloudout);
  std::cout<<trans.matrix()<<endl;
  viewer->removePointCloud("v1");
  PointCloudColorHandlerRGBField<PointT> rgb1(temp);
  viewer->addPointCloud<PointT>(temp, rgb1, "v1", v1);
  ui->qvtkWidget->update ();

}
void  Integrated::rSliderValueChanged (int value)
{
  roll = value/180.0*M_PI;
}
void  Integrated::pSliderValueChanged (int value)
{
  pitch = value/180.0*M_PI;
}
void  Integrated::ySliderValueChanged (int value)
{
  yaw = value/180.0*M_PI;
}

/*公共*/
void  Integrated::loadButtonPressed ()
{
     // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/cbc/图片/raw实验数据/1122/", tr ("Point cloud data (*.pcd *.ply)"));

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
  //*cloudout = *cloudin;
  pcl::copyPointCloud(*cloudin,*cloudout);

  viewPair();


}

void  Integrated::saveButtonPressed ()
{
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFile (filename.toStdString (), *cloudout);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloudout);
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
void  Integrated::cancelButtonPressed ()
{



}


void  Integrated::resetButtonPressed ()
{
    x1 = -2.0;  x2 = 2;  y1 = -2.0;  y2 = 2;  z1 = 0.0;  z2 = 3;  // The default value
    roll = 0;  pitch = 0; yaw = 0;
    lx = 1.0;   ly = 1.0;   lz = 1.0;
    MeanK = 50;  StddevMulThresh = 0.4;

    ui->doubleSpinBox_x1->setValue(x1);
    ui->doubleSpinBox_x2->setValue(x2);
    ui->doubleSpinBox_y1->setValue(y1);
    ui->doubleSpinBox_y2->setValue(y2);
    ui->doubleSpinBox_z1->setValue(z1);
    ui->doubleSpinBox_z2->setValue(z2);
    ui->doubleSpinBox_mean->setValue(MeanK);
    ui->doubleSpinBox_std->setValue(StddevMulThresh);
    ui->doubleSpinBox_1->setValue(lx);
    ui->doubleSpinBox_2->setValue(ly);
    ui->doubleSpinBox_3->setValue(lz);
    ui->spinBox_r->setValue(roll);
    ui->spinBox_p->setValue(pitch);
    ui->spinBox_y->setValue(yaw);

    rotate_flag=false;
    viewPair();
}

Integrated::~Integrated()
{
    delete ui;
}
