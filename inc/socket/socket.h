// Copyright 2020 by Hongbin Zhou
// Author: Hongbin Zhou

#ifndef __SOCKET_H__
#define __SOCKET_H__

#include <iostream>
#include <cstring>
#include <memory>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "image_msg.pb.h"

namespace tcp_socket {

#define SOCKET_SERVER false
#define SOCKET_CLIENT true
#define BOOST_ASIO boost::asio

class Socket
{

public:
  Socket();

  Socket(const bool& client_or_server,
         const std::string& ip_string, const int& port);

  ~Socket();

  size_t message_write(const std::string& write_buf);
  size_t message_exact_read(std::string& read_buf, const int& stream_size);
  size_t message_atleast_read(std::string& read_buf, const int& stream_size);

private:

  void initialize();

  std::shared_ptr<BOOST_ASIO::io_service> io_service_;

  std::shared_ptr<BOOST_ASIO::ip::tcp::socket> core_socket_;

  std::shared_ptr<BOOST_ASIO::ip::tcp::acceptor> server_accept_;

  std::shared_ptr<BOOST_ASIO::ip::tcp::endpoint> address_;

  //True for client and False for server
  //default is server
  const bool is_client_or_server_;
};

class Msg_image{

public:

  Msg_image();
  ~Msg_image();

  void set_image_msg(const cv::Mat &source_mat,
                     std::string& dst_bianry_str);

  void get_image_msg(cv::Mat &dst_mat,
                     const std::string& src_bianry_str);

  std::shared_ptr<image_msg::image>& get_msg_buf();

  double& time_stamp_update();

private:

  std::shared_ptr<image_msg::image> msg_buf_;

  boost::posix_time::ptime current_time_;
  boost::posix_time::ptime start_time_;
  double time_stamp;
};


class Msg_image_buf
{
public:
  Msg_image_buf();
  ~Msg_image_buf();

  void get_image_msg(cv::Mat &src_col, cv::Mat &src_disp,
                     const std::string& src_bianry_str);

  std::shared_ptr<image_msg::image_buf>& get_msg_buf();

  double& time_stamp_update();

private:

  std::shared_ptr<image_msg::image_buf> msg_buf_;

  boost::posix_time::ptime current_time_;
  boost::posix_time::ptime start_time_;
  double time_stamp;
};

}


#endif
