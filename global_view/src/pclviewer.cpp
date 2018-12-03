#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");
 
  initial();
  //viewPair();
 
  connect (ui->horizontalSlider_x1, SIGNAL (valueChanged (int)), this, SLOT (x1SliderValueChanged (int)));
  connect (ui->horizontalSlider_x2, SIGNAL (valueChanged (int)), this, SLOT (x2SliderValueChanged (int)));
  connect (ui->horizontalSlider_y1, SIGNAL (valueChanged (int)), this, SLOT (y1SliderValueChanged (int)));
  connect (ui->horizontalSlider_y2, SIGNAL (valueChanged (int)), this, SLOT (y2SliderValueChanged (int)));
  connect (ui->horizontalSlider_z1, SIGNAL (valueChanged (int)), this, SLOT (z1SliderValueChanged (int)));
  connect (ui->horizontalSlider_z2, SIGNAL (valueChanged (int)), this, SLOT (z2SliderValueChanged (int)));

  connect(ui->spinBox_x1, SIGNAL(valueChanged(int)), this, SLOT(updateLabelValue1(int)));
  connect(ui->spinBox_x2, SIGNAL(valueChanged(int)), this, SLOT(updateLabelValue2(int)));
  connect(ui->spinBox_y1, SIGNAL(valueChanged(int)), this, SLOT(updateLabelValue3(int)));
  connect(ui->spinBox_y2, SIGNAL(valueChanged(int)), this, SLOT(updateLabelValue4(int)));
  connect(ui->spinBox_z1, SIGNAL(valueChanged(int)), this, SLOT(updateLabelValue5(int)));
  connect(ui->spinBox_z2, SIGNAL(valueChanged(int)), this, SLOT(updateLabelValue6(int)));

  connect (ui->horizontalSlider_x1, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
  connect (ui->horizontalSlider_x2, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
  connect (ui->horizontalSlider_y1, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
  connect (ui->horizontalSlider_y2, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
  connect (ui->horizontalSlider_z1, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));
  connect (ui->horizontalSlider_z2, SIGNAL (sliderReleased ()), this, SLOT (conditFilter ()));

  connect (ui->spinBox_x1, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
  connect (ui->spinBox_x2, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
  connect (ui->spinBox_y1, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
  connect (ui->spinBox_y2, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
  connect (ui->spinBox_z1, SIGNAL(editingFinished()), this, SLOT (conditFilter()));
  connect (ui->spinBox_z2, SIGNAL(editingFinished()), this, SLOT (conditFilter()));  


  connect (ui->horizontalSlider_r, SIGNAL (valueChanged (int)), this, SLOT (rSliderValueChanged (int)));
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));
  connect (ui->horizontalSlider_y, SIGNAL (valueChanged (int)), this, SLOT (ySliderValueChanged (int)));
  connect (ui->spinBox_r, SIGNAL(valueChanged(int)), this, SLOT(r_updateLabelValue(int)));
  connect (ui->spinBox_p, SIGNAL(valueChanged(int)), this, SLOT(p_updateLabelValue(int)));
  connect (ui->spinBox_y, SIGNAL(valueChanged(int)), this, SLOT(y_updateLabelValue(int)));

  connect (ui->horizontalSlider_r, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
  connect (ui->horizontalSlider_p, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
  connect (ui->horizontalSlider_y, SIGNAL (sliderReleased ()), this, SLOT (rotate_UI ()));
  connect (ui->spinBox_r, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));
  connect (ui->spinBox_p, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));
  connect (ui->spinBox_y, SIGNAL(editingFinished()), this, SLOT (rotate_UI()));

  connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));     
  connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (loadButtonPressed()));
}
void PCLViewer::initial()
{ 
  x1 = -2.0;  x2 = 2;  y1 = -2.0;  y2 = 2;  z1 = 0.0;  z2 = 3;  // The default value   
  roll = 0;  pitch = 0; yaw = 0;
  cloudin.reset (new PointCloudT); // Setup the cloud pointer
  cloudout.reset (new PointCloudT); 
  temp.reset(new PointCloudT); 
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  v1=0;v2=0;
  // Set up the QVTK window
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  //ui->qvtkWidget->update ();
  viewer->initCameraParameters();  
  
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Before cond", 10, 10, "v1 text", v1);  
  viewer->addPointCloud(cloudin, "v1", v1);

  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
  viewer->addText("After cond", 10, 10, "v2 text", v2);  	
  viewer->addPointCloud(cloudin, "v2", v2);	

  viewer->addCoordinateSystem (2.5, "axis", v1);

  ui->qvtkWidget->update ();

  /*
  pointcloudname = QCoreApplication::arguments().at(1);
  pdname = pointcloudname.toUtf8().constData();//QString to String
  cout<<"文件名为："<<pdname<<endl;
  pcl::io::loadPCDFile(pdname,*cloudin);
  *cloudout = *cloudin;
  */
}

void PCLViewer::viewPair()
{  
  viewer->removePointCloud("v1");
  viewer->removePointCloud("v2");
  PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);
  PointCloudColorHandlerRGBField<PointT> rgb2(cloudout);
  viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);	
  viewer->addPointCloud<PointT>(cloudout, rgb2, "v2", v2);
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
  
}


Eigen::Matrix3d PCLViewer::euler2RotationMatrix(double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle*pitchAngle*rollAngle;
    Eigen::Matrix3d r = q.matrix();
    return r;
}

void PCLViewer::conditFilter()
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
    condrem.setInputCloud (cloudin);
    condrem.setKeepOrganized(true);
    condrem.filter (*cloudout);  
    
    viewer->removePointCloud("v2");
    PointCloudColorHandlerRGBField<PointT> rgb2(cloudout);  
    viewer->addPointCloud<PointT>(cloudout, rgb2, "v2", v2);
    ui->qvtkWidget->update ();

    std::cout<<"当前x,y,z范围分别为:"<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<" "<<z1<<" "<<z2<<std::endl;
    std::cout<<"has finished conditional filtering."<<std::endl;
  }
  
}
void  PCLViewer::rotate_UI()
{
  //Eigen::Matrix3d rotation_cal = euler2RotationMatrix(roll, pitch, yaw);
  //Eigen::Matrix4f trans = Eigen::Matrix4f::Identity(); 
  Eigen::Affine3f trans = Eigen::Affine3f::Identity();
  trans.rotate(Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
  trans.rotate(Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
  trans.rotate(Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ())); 
  
  pcl::transformPointCloud(*cloudin, *cloudin, trans);
  std::cout<<trans.matrix()<<endl;
  viewer->removePointCloud("v1");
  PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);  
  viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);
  ui->qvtkWidget->update ();

}

void  PCLViewer::loadButtonPressed ()
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

void  PCLViewer::saveButtonPressed ()
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

//显示小数
void PCLViewer::updateLabelValue1(int value)
{
    double doublevalue = value/100.0;
    ui->label_1->setText(QString::number(doublevalue, 'f', 2));
}
void PCLViewer::updateLabelValue2(int value)
{
    double doublevalue = value/100.0;
    ui->label_2->setText(QString::number(doublevalue, 'f', 2));
}
void PCLViewer::updateLabelValue3(int value)
{
    double doublevalue = value/100.0;
    ui->label_3->setText(QString::number(doublevalue, 'f', 2));
}
void PCLViewer::updateLabelValue4(int value)
{
    double doublevalue = value/100.0;
    ui->label_4->setText(QString::number(doublevalue, 'f', 2));
}
void PCLViewer::updateLabelValue5(int value)
{
    double doublevalue = value/100.0;
    ui->label_5->setText(QString::number(doublevalue, 'f', 2));
}
void PCLViewer::updateLabelValue6(int value)
{
    double doublevalue = value/100.0;
    ui->label_6->setText(QString::number(doublevalue, 'f', 2));
}


void PCLViewer::x1SliderValueChanged (int value)
{
  x1 = value/100.0;
  printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void PCLViewer::x2SliderValueChanged (int value)
{
  x2 = value/100.0;
  printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void PCLViewer::y1SliderValueChanged (int value)
{
  y1 = value/100.0;
  printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void PCLViewer::y2SliderValueChanged (int value)
{
  y2 = value/100.0;
  printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void PCLViewer::z1SliderValueChanged (int value)
{
  z1 = value/100.0;
  printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void PCLViewer::z2SliderValueChanged (int value)
{
  z2 = value/100.0;
  printf ("SliderValueChanged: [ %f | %f | %f | %f | %f | %f ]\n", x1, x2, y1, y2, z1, z2);
}

void  PCLViewer::rSliderValueChanged (int value)
{
  roll = value/180.0*M_PI;
}
void  PCLViewer::pSliderValueChanged (int value)
{
  pitch = value/180.0*M_PI;
}
void  PCLViewer::ySliderValueChanged (int value)
{
  yaw = value/180.0*M_PI;
}

void  PCLViewer::r_updateLabelValue(int value)
{
  //double doublevalue = value/100.0;
  ui->label_r->setText(QString::number(value));
}
void  PCLViewer::p_updateLabelValue(int value)
{
  ui->label_p->setText(QString::number(value));
}
void  PCLViewer::y_updateLabelValue(int value)
{
  ui->label_y->setText(QString::number(value));
}


PCLViewer::~PCLViewer ()
{
  delete ui;
}
