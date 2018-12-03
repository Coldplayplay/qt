
//#include <ui_manual_registration.h>

// QT4
#include <QMainWindow>
#include <QMutex>
#include <QTimer>

// Boost
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

// PCL
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/registration/transformation_estimation_svd.h>

typedef pcl::PointXYZRGBA PointT;

// Useful macros
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

namespace Ui
{
  class MainWindow;
}

class ManualRegistration : public QMainWindow
{
  Q_OBJECT
  public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    ManualRegistration ();

    ~ManualRegistration ()
    {
    }

    void
    setSrcCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src)
    {
      cloud_src_ = cloud_src;
      cloud_src_present_ = true;
    }
    void
    setDstCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_dst)
    {
      cloud_dst_ = cloud_dst;
      cloud_dst_present_ = true;
    }

    void
    SourcePointPickCallback (const pcl::visualization::PointPickingEvent& event, void*);
    void
    DstPointPickCallback (const pcl::visualization::PointPickingEvent& event, void*);

  protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_src_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_dst_;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_dst_;

    QMutex mtx_;
    QMutex vis_mtx_;
    Ui::MainWindow *ui_;
    QTimer *vis_timer_;

    bool                              cloud_src_present_;
    bool                              cloud_src_modified_;
    bool                              cloud_dst_present_;
    bool                              cloud_dst_modified_;

    bool                              src_point_selected_;
    bool                              dst_point_selected_;

    pcl::PointXYZ                     src_point_;
    pcl::PointXYZ                     dst_point_;

    pcl::PointCloud<pcl::PointXYZ>    src_pc_;
    pcl::PointCloud<pcl::PointXYZ>    dst_pc_;

    Eigen::Matrix4f                   transform_;

  public Q_SLOTS:
    void 
    confirmSrcPointPressed();
    void 
    confirmDstPointPressed();
    void 
    calculatePressed();
    void
    clearPressed();
    void 
    orthoChanged(int state);
    void 
    applyTransformPressed();
    void
    refinePressed();
    void
    undoPressed();
    void
    safePressed();

  private Q_SLOTS:
    void
    timeoutSlot();

};
