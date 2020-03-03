#include "log.h"
#include "disparty2pcl/disparity_to_pcl.h"

using disparitytopcl::Dispt_pcl;

int main(int argc, char **argv)
{

  FLAGS_log_dir = "../log";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("---Convert disparity to point cloud---");

  if(argc < 3){

    LOG_OUTPUT("error! need input for diparity and color image path");
    return -1;
  }

  std::shared_ptr<Dispt_pcl> disp_pcl = std::make_shared<Dispt_pcl>();

  disp_pcl->work_flow(argv[1], argv[2]);

  google::ShutdownGoogleLogging();

  return 0;
}
