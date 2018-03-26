#include <iostream>
#include <boost/python.hpp>
using namespace std;
using namespace boost::python;

int main()
{
      Py_Initialize();
      if(!Py_IsInitialized())
      {
          std::cout <<"1" <<std::endl;
          return -1;
      }
      PyRun_SimpleString("import sys");
      int argc = 2;
      char *argv[2];
      argv[0] = "/home/cbc/DL/PointNet/pointnet/sem_seg/model.py";
      argv[1] = "/home/cbc/pcl_without_ros/qt/test_python/test.py";
      PySys_SetArgv(argc, argv);

/*
      //python3的用法
      if(PyRun_SimpleString("exec(open('/home/cbc/pcl_without_ros/qt/test_python/test.py').read())") == NULL)
      {
          std::cout <<"2" <<std::endl;
          return -1;
      }
  
      //测试程序
      if(PyRun_SimpleString("execfile('/home/cbc/pcl_without_ros/qt/test_python/test.py')") == NULL)
      {
          std::cout <<"3" <<std::endl;
          return -1;
      }
  */    
   
   
      if(PyRun_SimpleString("execfile('/home/cbc/DL/PointNet/pointnet/sem_seg/model.py')") == NULL)
      {
          std::cout <<"3" <<std::endl;
          return -1;
      }

      std::cout<<"mistake."<<endl;
      Py_Finalize();
      return 0;
}
