#include "semseg.h"
//#include "ui_semseg.h"
#include "../build/ui_semseg.h"

SemSeg::SemSeg(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SemSeg)
{
    ui->setupUi(this);
    initial();

    connect (ui->pushButton_semseg, SIGNAL (clicked ()), this, SLOT (semsegButtonPressed()));
    connect (ui->pushButton_openobj, SIGNAL (clicked()), this, SLOT (obj_open()));
}


void SemSeg::initial()
{
  cloud1.reset (new PointCloudT); // Setup the cloud pointer
  cloud2.reset (new PointCloudT);

  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  v1=0;v2=0;
  // Set up the QVTK window
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  viewer->initCameraParameters();

  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("semantic result", 10, 10, "v1 text", v1);
  viewer->addPointCloud(cloud1, "v1", v1);

  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0, 0, 0, v2);
  viewer->addText("3D box", 10, 10, "v2 text", v2);
  viewer->addPointCloud(cloud2, "v2", v2);
  ui->qvtkWidget->update();

}




void SemSeg::obj_open()
{
    // You might want to change "/home/" if you're not on an *nix platform
     QString filename = QFileDialog::getOpenFileName (this, tr ("Open obj data"), "/home/cbc/DL/PointNet/pointnet/sem_seg", tr ("Obj data (*.obj)"));

     PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

     if (filename.isEmpty ())
       return;

     obj2pcd(filename.toStdString ().c_str ());
     viewer->updatePointCloud(cloud1, "v1");
     ui->qvtkWidget->update();


}

void SemSeg::obj2pcd(string filename)
{
    PointT point;

    ifstream infile(filename, ios::in);
    string delim(" ");
    string textline;

    cloud1.reset(new PointCloudT);

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
                cloud1->push_back(point);
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


void SemSeg::semsegButtonPressed()
{
  Py_Initialize();
  if(!Py_IsInitialized())
  {
      std::cout <<"failure" <<std::endl;
      return;
  }
  PyRun_SimpleString("import sys");
  int argc = 1;
  char *argv[1];
  argv[0] = "/home/cbc/DL/PointNet/pointnet/sem_seg/test_2.py";

  PySys_SetArgv(argc, argv);

  if(PyRun_SimpleString("execfile('/home/cbc/DL/PointNet/pointnet/sem_seg/test_2.py')") == NULL)
  {
      std::cout <<"Success test semseg." <<std::endl;
      return;
  }

  std::cout<<"Mistake."<<endl;
  Py_Finalize();
  return;


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




SemSeg::~SemSeg()
{
    delete ui;
}
