#include "log.h"
#include "visualizer/pcl_visualizer.h"

#include <chrono>
#include <thread>

namespace pcl_visualizer {

void pcl_visualizer_thread(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                           const uint8_t& flag){

  LOG_OUTPUT("run visualier!");

  std::shared_ptr<pcl_visualizer::Pcl_viewer> pcl_viewer =
      std::make_shared<pcl_visualizer::Pcl_viewer>();

  while( points->size() == 0){

    LOG_OUTPUT("zero points! waiting for points");

  }

  pcl_viewer->run_viewer(points, flag);

  LOG_OUTPUT("visualier exit!");

}

Pcl_viewer::Pcl_viewer(){

  pcl_viewer = std::make_shared<pcl::visualization::PCLVisualizer>();

  pcl_viewer->setBackgroundColor(0, 0, 0);
  pcl_viewer->addCoordinateSystem(1.0);
  pcl_viewer->initCameraParameters();

}

Pcl_viewer::~Pcl_viewer(){

}

inline void Pcl_viewer::add_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
                                   const std::string& id,
                                   const double& value){

  pcl_viewer->addPointCloud<pcl::PointXYZ>(points, id);

  pcl_viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, id);
}

inline void Pcl_viewer::add_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& points,
                                   const std::string& id,
                                   const double& value){

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(points);

  pcl_viewer->addPointCloud<pcl::PointXYZRGB>(points, rgb, id);

  pcl_viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, id);
}

void Pcl_viewer::update_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
                               const std::string &id,
                               const double &value){

//  pcl_viewer->removePointCloud(id);
//  add_points(points, id, value);
  pcl_viewer->updatePointCloud(points, id);

}

void Pcl_viewer::update_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                               const std::string &id,
                               const double &value){

//  pcl_viewer->removePointCloud(id);
//  add_points(points, id, value);
  pcl_viewer->updatePointCloud(points, id);

}

void Pcl_viewer::run_viewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
                            const std::string &id,
                            const double &value){

  while(!pcl_viewer->wasStopped()){

    pcl_viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    update_points(points, id, value);
  }
}

void Pcl_viewer::run_viewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                            const uint8_t& flag,
                            const std::string &id,
                            const double &value){

  add_points(points, id, value);

  while(!pcl_viewer->wasStopped()){

    if(flag){
     update_points(points, id, value);
    }

    pcl_viewer->spinOnce(100);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}
