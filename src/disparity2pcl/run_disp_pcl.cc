#include "log.h"
#include "disparty2pcl/disparity_to_pcl.h"
#include <pcl/range_image/range_image.h>

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

  disp_pcl->disparity_image_process(argv[1]);

  disp_pcl->add_color_image(argv[2]);

  disp_pcl->convert_pointcloud();

  disp_pcl->viewer();

  google::ShutdownGoogleLogging();

  return 0;
}
