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

  connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));     
  connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (loadButtonPressed()));
}
void PCLViewer::initial()
{ 
  x1 = -0.5;  x2 = 0.5;  y1 = -0.4;  y2 = 0.25;  z1 = 0.0;  z2 = 1.2;  // The default value   
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
//涂成单一颜色  
  PointCloudColorHandlerCustom<PointT>  green(cloudin, 0, 255, 0);	
  PointCloudColorHandlerCustom<PointT>  blue(cloudout, 0, 0, 255); 
  viewer->addPointCloud(cloudin, green, "v1", v1);	
  viewer->addPointCloud(cloudout, blue, "v2", v2);   
  
//原色
  //PointCloudColorHandlerRGBField<PointT> rgb1(cloudin);
 // PointCloudColorHandlerRGBField<PointT> rgb2(cloudin);
  //viewer->addPointCloud<PointT>(cloudin, rgb1, "v1", v1);	
  //viewer->addPointCloud<PointT>(cloudout, rgb2, "v2", v2); 

  
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
  
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
  
    PointCloudColorHandlerCustom<PointT> blue(cloudout, 0, 0, 255);
    viewer->removePointCloud("v2");
    viewer->addPointCloud(cloudout, blue, "v2", v2);
    ui->qvtkWidget->update ();

    std::cout<<"当前x,y,z范围分别为:"<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<" "<<z1<<" "<<z2<<std::endl;
    std::cout<<"has finished conditional filtering."<<std::endl;
  }
  
}
//voxelFilter(cloudin,cloudout,leafsize)
//{

//}

void  PCLViewer::loadButtonPressed ()
{
     // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

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

PCLViewer::~PCLViewer ()
{
  delete ui;
}
