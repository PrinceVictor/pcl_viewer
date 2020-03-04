#include "log.h"
#include "disparty2pcl/disparity_to_pcl.h"
#include "visualizer/pcl_visualizer.h"

#include <thread>

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

  LOG_OUTPUT("run disparity!");

  std::shared_ptr<Dispt_pcl> disp_pcl = std::make_shared<Dispt_pcl>();

  disp_pcl->work_flow(argv[1], argv[2]);

  std::thread pcl_thread(&pcl_visualizer::pcl_visualizer_thread, std::ref(disp_pcl->get_point_cloud()));

  std::this_thread::sleep_for(std::chrono::seconds(5));

  disp_pcl->work_flow(argv[3], argv[4]);

  pcl_thread.join();

  google::ShutdownGoogleLogging();

  return 0;
}



