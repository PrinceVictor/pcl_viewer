#ifndef __DISPARITY_TO_PCL__
#define __DISPARITY_TO_PCL__

#include "log.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <iostream>
#include <vector>
#include <cstring>
#include <memory>

namespace disparitytopcl {

class Dispt_pcl {

public:

  Dispt_pcl();

  ~Dispt_pcl();

  void disparity_image_process(const std::string& image_path);

  void add_color_image(const std::string& image_path);

  void convert_pointcloud();

  void viewer();

private:

  void camera_parameter_init();

  void convert_cv2eigen();

  std::shared_ptr<Eigen::MatrixXd> disparity_matrix_;

  cv::Mat color_image_;
  cv::Mat disparity_image_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;

  Eigen::Matrix3d camera_intrinsics_k_;
  double baseline_fb_;

};



}


#endif
