#include <iostream>

#include <log.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char *argv[])
{

  FLAGS_log_dir = "../log";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("This is a pcl viewer test");

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud);

  while (!viewer.wasStopped())
  {
  }

  google::ShutdownGoogleLogging();

  return 0;
}
