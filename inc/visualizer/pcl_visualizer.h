#ifndef __PCL_VISUALIZER_H__
#define __PCL_VISUALIZER_H__

#include <pcl/visualization/pcl_visualizer.h>

#include <memory>

namespace pcl_visualizer {

void pcl_visualizer_thread(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                           const uint8_t& flag);

class Pcl_viewer{

public:

  Pcl_viewer();

  ~Pcl_viewer();

  void run_viewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
                  const std::string &id = "points",
                  const double &value = 1);

  void run_viewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                  const uint8_t &flag = 0,
                  const std::string &id = "points",
                  const double &value = 1);

  void test(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points);

private:

  void add_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
                  const std::string& id,
                  const double& value = 1);

  void add_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& points,
                  const std::string& id,
                  const double& value);

  void update_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
                     const std::string& id,
                     const double& value);

  void update_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& points,
                     const std::string& id,
                     const double& value);

  std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;

};

}



#endif
