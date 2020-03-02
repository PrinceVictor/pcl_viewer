#include "log.h"
#include "disparty2pcl/disparity_to_pcl.h"


int main(int argc, char **argv)
{

  FLAGS_log_dir = "../log";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("---Convert disparity to point cloud---");



//  pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

//  std::string file = argv[1];
//  pcl::io::loadPCDFile(file, *points);

  google::ShutdownGoogleLogging();

  return 0;
}
