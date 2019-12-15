#include <log.h>
#include <visualizer/visualizer.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <iostream>

int main(int argc, char *argv[])
{

  //  FLAGS_log_dir = "../log";
  FLAGS_log_dir = "/home/victor/mobile_robot/pcl_viewer/pointcloud_view/log";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("---This is a point cloud viewer, supported by opengl---");


  google::ShutdownGoogleLogging();

  return 0;
}
