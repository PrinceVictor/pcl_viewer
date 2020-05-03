#include "log.h"
#include "disparty2pcl/disparity_to_pcl.h"
#include "visualizer/pcl_visualizer.h"
#include "socket/socket.h"

#include <thread>

using disparitytopcl::Dispt_pcl ;
using tcp_socket::Socket;
using tcp_socket::Msg_image_buf;


int main(int argc, char **argv)
{

  FLAGS_log_dir = "../log";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("---Convert disparity to point cloud---");

//  if(argc < 3){

//    LOG_OUTPUT("error! need input for diparity and color image path");
//    return -1;
//  }

  LOG_OUTPUT("run disparity!");

  std::shared_ptr<Socket> socket_ptr =
      std::make_shared<Socket>(SOCKET_SERVER, "127.0.0.1", 1080);

  std::shared_ptr<Msg_image_buf> image_msg_ptr = std::make_shared<Msg_image_buf>();

  std::shared_ptr<Dispt_pcl> disp_pcl = std::make_shared<Dispt_pcl>();

  std::thread pcl_thread(&pcl_visualizer::pcl_visualizer_thread,
                         std::ref(disp_pcl->get_point_cloud()),
                         std::ref(disp_pcl->get_flag()));

  std::string temp = "good11";
  socket_ptr->message_write(temp);

  int steps = 1;

  while(true){

    LOG_OUTPUT("verifing----- steps {}", steps);

    std::string read_str;
    size_t length = socket_ptr->message_exact_read(read_str, 6);
    LOG_OUTPUT("length {} size {}", length, read_str.size());
    if(length != 6 || read_str != "start1"){

      LOG_OUTPUT("verified failed");
      continue;
    }


    LOG_OUTPUT("verified successfully");
    //    image_msg_ptr->time_stamp_update();

    //    size_t
    //    length = socket_ptr->message_exact_read(read_str, 1397275);
    length = socket_ptr->message_exact_read(read_str, 1813562);

    LOG_OUTPUT("length {} size {}", length, read_str.size());

    cv::Mat color, disp;
    image_msg_ptr->get_image_msg(color, disp, read_str);

    LOG_OUTPUT("time stamp {} current {}",
               image_msg_ptr->get_msg_buf()->image_(0).time_stamp(),
               image_msg_ptr->time_stamp_update());

    disp_pcl->work_flow(disp, color);

    steps++;
    // cv::imshow("color", color);
    // cv::imshow("disp", disp);
    // cv::waitKey(0);

  }
//  std::this_thread::sleep_for(std::chrono::seconds(5));

//  disp_pcl->work_flow(argv[3], argv[4]);

  pcl_thread.join();

  google::ShutdownGoogleLogging();

  return 0;
}



