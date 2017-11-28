#include "statisticfilter.h"
#include "../build/ui_statisticfilter.h"

StatisticFilter::StatisticFilter(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::StatisticFilter)
{
    ui->setupUi(this);
    this->setWindowTitle ("Statistic Filter");
    initial();

    connect (ui->spinBox_1, SIGNAL (valueChanged (int)), this, SLOT (updateLabelValue(int)));
    connect (ui->spinBox_1, SIGNAL (valueChanged (int)), this, SLOT (valueChanged1(int)));
    connect (ui->spinBox_2, SIGNAL (valueChanged (int)), this, SLOT (valueChanged2(int)));
    connect (ui->spinBox_1, SIGNAL(editingFinished()), this, SLOT (statisticFilter()));
    connect (ui->spinBox_2, SIGNAL(editingFinished()), this, SLOT (statisticFilter()));
    connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));
    connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (loadButtonPressed()));

}

void StatisticFilter::initial()
{
  MeanK = 50;
  StddevMulThresh = 0.4;
  cloudin.reset (new PointCloudT); // Setup the cloud pointer
  cloudout.reset (new PointCloudT);
 // temp.reset(new PointCloudT);
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  v1=0;v2=0;
  // Set up the QVTK window
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  //ui->qvtkWidget->update ();
  viewer->initCameraParameters();

  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Before statis filter", 10, 10, "v1 text", v1);
  viewer->addPointCloud(cloudin, "v1", v1);

  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0, 0, 0, v2);
  viewer->addText("After statis filter", 10, 10, "v2 text", v2);
  viewer->addPointCloud(cloudout, "v2", v2);
  ui->qvtkWidget->update ();

}



void StatisticFilter::viewPair()
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

void StatisticFilter::statisticFilter()
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloudin);
    sor.setMeanK (MeanK);
    sor.setStddevMulThresh (StddevMulThresh);
    sor.filter (*cloudout);

    std::cout<<"离群点去除前，点云数目："<<cloudin->points.size()<<endl;
    std::cout<<"离群点去除后，点云数目："<<cloudout->points.size()<<endl;    

    viewer->removePointCloud("v2");
    PointCloudColorHandlerRGBField<PointT> rgb2(cloudout);    
    viewer->addPointCloud<PointT>(cloudout, rgb2, "v2", v2);    
    ui->qvtkWidget->update ();

}

void StatisticFilter::loadButtonPressed()
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

void  StatisticFilter::saveButtonPressed ()
{
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/cbc/图片/", tr ("Point cloud data (*.pcd *.ply)"));

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

void StatisticFilter::valueChanged1(int value)
{
    MeanK = value;
}

void StatisticFilter::valueChanged2(int value)
{
    StddevMulThresh = value/100.0;
}

//显示小数
void StatisticFilter::updateLabelValue(int value)
{
    double doublevalue = value/100.0;
    ui->label_num->setText(QString::number(doublevalue, 'f', 2));
}

StatisticFilter::~StatisticFilter()
{
    delete ui;
}
