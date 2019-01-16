#include "semseg.h"
#include "build/ui_semseg.h"

SemSeg::SemSeg(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SemSeg)
{
    ui->setupUi(this);
    this->setWindowTitle ("Robot Grab");
    initial();

    connect (ui->pushButton_load, SIGNAL (clicked()), this, SLOT (loadButtonPressed()));
    connect (ui->pushButton_semseg, SIGNAL (clicked ()), this, SLOT (semsegButtonPressed()));
    connect (ui->pushButton_show, SIGNAL (clicked ()), this, SLOT (showButtonPressed()));
    connect (ui->checkBox_cube, SIGNAL(stateChanged(int)), this, SLOT(checkBox_cubeChanged(int)));
    connect (ui->pushButton_pose, SIGNAL (clicked ()), this, SLOT (poseButtonPressed()));
    connect (ui->pushButton_send, SIGNAL (clicked ()), this, SLOT (sendButtonPressed()));

    connect (ui->spinBox_id, SIGNAL (valueChanged (QString)), this, SLOT (idSpinBoxValueChanged(QString)));
    connect (ui->lineEdit_ip, SIGNAL (textChanged (QString)), this, SLOT (ipLineEditValueChanged(QString)));
}


void SemSeg::initial()
{

  //id=1;
  show_cube = true;
  toBeSegFileName="/home/cbc/DL/semantic_segmantation3D/PointNet/pointnet/data/5081_test/bishe.txt";
  segedFileName="/home/cbc/DL/semantic_segmantation3D/PointNet/pointnet/sem_seg/test_5081/dump/bishe_pred.obj";
  cloud1.reset (new PointCloudT);
  cloud2.reset (new PointCloudLT);
  cloud3.reset (new PointCloudT);
  cloud_tmp.reset(new PointCloudT);
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  v1=0;v2=0;
  // Set up the QVTK window
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  viewer->initCameraParameters();

  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(255, 255, 255, v1);
  viewer->addText("raw", 10, 10, 20,0,0,0,"v1 text", v1);
  viewer->addPointCloud(cloud1, "v1", v1);

  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(255, 255, 255, v2);
  viewer->addText("result", 10, 10, 20,0,0,0,"v2 text", v2);
  viewer->addPointCloud(cloud3, "v2", v2);
  ui->qvtkWidget->update();


}




void SemSeg::loadButtonPressed()
{
     filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/cbc/图片/processed实验数据/", tr ("Point cloud data (*.pcd *.ply)"));

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
       pcl::copyPointCloud (*cloud_tmp, *cloud1);
     else
     {
       PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
       std::vector<int> vec;
       pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud1, vec);
     }

     //pcl::copyPointCloud(*cloud1,*cloud2);

     //viewPair();
     viewer->removePointCloud("v1");
     PointCloudColorHandlerRGBField<PointT> rgb1(cloud1);
     viewer->addPointCloud<PointT>(cloud1, rgb1, "v1", v1);     
     ui->qvtkWidget->update ();

     ui->lineEdit_status->setText("None.");

}
void SemSeg::semsegButtonPressed()
{
    pcd2txt();

    Py_Initialize();
      if(!Py_IsInitialized())
      {
          std::cout <<"failure" <<std::endl;
          return;
      }
      PyRun_SimpleString("import sys");
      int argc = 1;
      char *argv[1];
      argv[0] = "/home/cbc/DL/semantic_segmantation3D/PointNet/pointnet/sem_seg/test.py";

      PySys_SetArgv(argc, argv);

      if(PyRun_SimpleString("execfile('/home/cbc/DL/semantic_segmantation3D/PointNet/pointnet/sem_seg/test_bishe.py')") == NULL)
      {
          std::cout <<"Success semseg." <<std::endl;
          QString text="Success semseg.";
          ui->lineEdit_status->setText(text);
          return;
      }
      else
        std::cout<<"Mistake."<<endl;
      Py_Finalize();
      return;


}
void SemSeg::showButtonPressed()
{
    //obj2pcd();//如果用深度学习分割的结果
    pcl::io::loadPCDFile ("/home/cbc/daifenge_annotated.pcd", *cloud2);
    pcl::copyPointCloud (*cloud2, *cloud3);
    viewer->updatePointCloud(cloud3, "v2");
    viewer->resetCamera ();
    ui->qvtkWidget->update();
}
void SemSeg::checkBox_cubeChanged(int value)
{
    show_cube = value;
    if(!show_cube)
    {
        for(int i=0; i<3; i++)
        {
            stringstream ss;
            ss<<"cube"<<i;
            viewer->removeShape(ss.str());
        }
    }
    else
        poseButtonPressed();

}
void SemSeg::poseButtonPressed()
{
    xuan.clear();
    ping.clear();
    PointCloudT proj;
    pcl::PCA<PointT> pca;

    double a[10] = {1.0, 0.0, 0.0, 0.2, 0.4, 0.6};
    double b[10] = {0.0, 1.0, 0.0, 0.2, 0.4, 0.6};
    double c[10] = {0.0, 0.0, 1.0, 0.2, 0.4, 0.6};

    //int i = 0;
    int labelnum[4]={2,11,5,9};
    for(int i=0;i<3;i++)
    {
        PointCloudLT::Ptr cloudsingle (new PointCloudLT);
        for (size_t j=0; j<cloud2->points.size(); j++)
        {
          if(cloud2->points[j].label==labelnum[i])//避雷器
            cloudsingle->push_back(cloud2->points[j]);
        }
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
        pcl::PointCloud<PointLT> cPoints;
        pcl::transformPointCloud(*cloudsingle, cPoints, p2w);

        PointLT min_pt, max_pt;
        pcl::getMinMax3D(cPoints, min_pt, max_pt);
        Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

        // final transform
        Eigen::Quaternionf qfinal(eigDx);
        Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();
        xuan.push_back(qfinal);
        ping.push_back(tfinal);
        // draw the cloud and the box
        stringstream ss;
        ss << "cube" << i;

        cout << "当前id为：" << ss.str() << endl;
        cout << "旋转四元数：["<<qfinal.x()<<", "<<qfinal.y()<<", "<<qfinal.z()<<", "<<qfinal.w()<<"]"<<endl;
        cout << "平移向量：["<<tfinal[0]<<", "<<tfinal[1]<<", "<<tfinal[2]<<"]"<<endl;
        cout<<"min_pt:"<<"("<<min_pt.x<<", "<<min_pt.y<<", "<<min_pt.z<<")"<<endl;
        cout<<"max_pt:"<<"("<<max_pt.x<<", "<<max_pt.y<<", "<<max_pt.z<<")"<<endl;
        cout << endl;
        if(show_cube)
        {
         viewer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z, ss.str(), v2);
        //viewer->addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z, a[i], b[i], c[i], ss.str(), v2);
        //viewer->addText(ss.str(), 20, 20, a[i], b[i], c[i], ss1.str(), v2);
         ui->qvtkWidget->update();
        }

    }
}

void SemSeg::sendButtonPressed()
{

    QString text="success.";
    ui->lineEdit_send->setText(text);
}

void SemSeg::idSpinBoxValueChanged(QString value)
{
    string ids = value.toStdString();
    int id = stoi(ids);
    for(int i=0;i<3;i++)
    {
        if(id==i+1)
        {
            /*
            Eigen::Matrix3f R1 = xuan[i].normalized().toRotationMatrix();//四元数--->矩阵
            cv::Mat R_m(3, 3, CV_64FC1); 
	    cv::Mat r_mo;   
	    cv::eigen2cv(R1, R_m);    
	    cv::Rodrigues(R_m, r_mo);//矩阵2向量
            
            stringstream ss_rpy;
            ss_rpy << "["<<r_mo.at<double>(0)<<", "<<r_mo.at<double>(1)<<", "<<r_mo.at<double>(2)<<"]";
            */
            
            stringstream ss_xyz;
       	    ss_xyz <<fixed<<setprecision(3)<< ping[i][0]<<", "<<ping[i][1]<<", "<<ping[i][2];	
       	    stringstream ss_rpy;
       	    ss_rpy <<fixed<<setprecision(3)<< xuan[i].x()<<", "<<xuan[i].y()<<", "<<xuan[i].z()<<", "<<xuan[i].w();	      
            
            QString xyz=QString::fromStdString(ss_xyz.str());
            QString rpy=QString::fromStdString(ss_rpy.str());

            ui->lineEdit_xyz->setText(xyz);
            ui->lineEdit_rpy->setText(rpy);
            break;
        }

    }

}
void SemSeg::ipLineEditValueChanged(QString value)
{

}

void SemSeg::pcd2txt()
{
    ofstream out(toBeSegFileName);

      if(out.is_open())
      {
        for(int i=0; i<cloud1->size(); i++)
        {
          float x = cloud1->points[i].x;
          float y = cloud1->points[i].y;
          float z = cloud1->points[i].z;
          int r = cloud1->points[i].r;
          int g = cloud1->points[i].g;
          int b = cloud1->points[i].b;

          out<<x<<" ";
          out<<y<<" ";
          out<<z<<" ";
          out<<r<<" ";
          out<<g<<" ";
          out<<b<<" ";

          out<<"\n";

        }
        out.close();
      }

}
void SemSeg::split(const string& src, const string& delim, vector<string>& dest)
{
    string str = src;
    string::size_type start = 0, index;
    string substr;

    index = str.find_first_of(delim, start);    //在str中查找(起始：start) delim的任意字符的第一次出现的位置
    while(index != string::npos)
    {
        substr = str.substr(start, index-start);
        dest.push_back(substr);
        start = str.find_first_not_of(delim, index);    //在str中查找(起始：index) 第一个不属于delim的字符出现的位置
        if(start == string::npos) return;

        index = str.find_first_of(delim, start);
    }
    substr = str.substr(start, index-start);
    dest.push_back(substr);
}
void SemSeg::obj2pcd()
{
    PointT point;

    ifstream infile(segedFileName, ios::in);
    string delim(" ");
    string textline;    

    if(infile.good())
    {
        while(!infile.fail())
        {
            vector<string> line;
            getline(infile, textline);
            split(textline, delim, line);

            if(line.size()>6)
            {
                point.x = stod(line[1]);
                point.y = stod(line[2]);
                point.z = stod(line[3]);
                point.r = stoi(line[4]);
                point.g = stoi(line[5]);
                point.b = stoi(line[6]);
                cloud3->push_back(point);
            }
        }
        cout<<"succeed in transforming obj into pcd."<<endl;
    }
    else
    {
        cout<<"wrong obj file."<<endl;
    }
    infile.close();

}
void SemSeg::viewPair()
{
  viewer->removePointCloud("v1");
  viewer->removePointCloud("v2");

  PointCloudColorHandlerRGBField<PointT> rgb1(cloud1);
  PointCloudColorHandlerRGBField<PointT> rgb2(cloud3);

  viewer->addPointCloud<PointT>(cloud1, rgb1, "v1", v1);
  viewer->addPointCloud<PointT>(cloud3, rgb2, "v2", v2);

  viewer->resetCamera ();
  ui->qvtkWidget->update ();

}


SemSeg::~SemSeg()
{
    delete ui;
}
