#include "disparty2pcl/disparity_to_pcl.h"

#include <opencv2/core/eigen.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
#include <mutex>
#include <cmath>
#include <boost/make_shared.hpp>

namespace disparitytopcl {

Dispt_pcl::Dispt_pcl(){

#ifdef USE_DEBUG

  LOG_OUTPUT("DEBUG is on !!!");
#endif

  camera_parameter_init();

}

Dispt_pcl::~Dispt_pcl(){

}

void Dispt_pcl::work_flow(const std::string& image_path1,
                          const std::string& image_path2){

  std::chrono::steady_clock::time_point time_point =
      std::chrono::steady_clock::now();

  disparity_image_process(image_path1);

  add_color_image(image_path2);

  double load_image = std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::steady_clock::now() - time_point).count();
  LOG_OUTPUT("load image took {} ms", load_image );

  create_pointcloud();

  double resize_pointcloud = std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::steady_clock::now() - time_point).count();
  LOG_OUTPUT("resize num of {} points took {} ms",
             disparity_image_.cols * disparity_image_.rows,
             resize_pointcloud-load_image );

  convert_pointcloud();

  double conver_pcl = std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::steady_clock::now() - time_point).count();
  LOG_OUTPUT("convert into point cloud took {} ms", conver_pcl-resize_pointcloud );

//  viewer();
}

void Dispt_pcl::camera_parameter_init(){

  camera_intrinsics_k_ << 721.5377, 0, 609.5593,
                          0, 721.5377, 172.854,
                          0, 0, 1;

  LOG_OUTPUT("camera intrinsics {:<9.2f} {:<9.2f} {:<9.2f}", camera_intrinsics_k_(0,0),
                                                             camera_intrinsics_k_(0,1),
                                                             camera_intrinsics_k_(0,2));
  LOG_OUTPUT("                  {:<9.2f} {:<9.2f} {:<9.2f}", camera_intrinsics_k_(1,0),
                                                             camera_intrinsics_k_(1,1),
                                                             camera_intrinsics_k_(1,2));
  LOG_OUTPUT("                  {:<9.2f} {:<9.2f} {:<9.2f}", camera_intrinsics_k_(2,0),
                                                             camera_intrinsics_k_(2,1),
                                                             camera_intrinsics_k_(2,2));

  Eigen::Vector3d trans_rect02, trans_rect03;

  trans_rect02 << 4.485728e1, 2.163791e-1, 2.745884e-3;
  trans_rect03 << -3.395242e2, 2.199936e0, 2.729905e-3;

  //according to the kitti coordinate systems
  //we have T_rect02 = K*Trans_02 to transform world in to image plane
  //here, our goal is to get the baseline bewtween 02 and 03
  //-trans_rect02 + trans_rect03 to trans from 02 into 03

  Eigen::Vector3d trans_23 = -trans_rect02 + trans_rect03;
  baseline_fb_ = std::abs(trans_23(0));

  LOG_OUTPUT("baseline23 {:.2f}", baseline_fb_);

}

inline void Dispt_pcl::disparity_image_process(const std::string& image_path){

  disparity_image_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);

  LOG_OUTPUT("read disparity image from {}", image_path.c_str());
  LOG_OUTPUT("image channel {} size x {} y {} type {}",
             disparity_image_.channels(),
             disparity_image_.cols,
             disparity_image_.rows,
             disparity_image_.type());

#ifdef USE_DEBUG
  disparity_matrix_ = std::make_shared<Eigen::MatrixXd>(disparity_image_.rows,
                                                        disparity_image_.cols);

  cv::cv2eigen(disparity_image_, *disparity_matrix_);

  *disparity_matrix_ = disparity_matrix_->array().inverse() * baseline_fb_;

  LOG_OUTPUT("matrix size {}", disparity_matrix_->size());

#endif



}

inline void Dispt_pcl::add_color_image(const std::string &image_path){

  color_image_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);

  LOG_OUTPUT("read color image from {}", image_path.c_str());
  LOG_OUTPUT("image channel {} size x {} y {} type {}",
             color_image_.channels(),
             color_image_.cols,
             color_image_.rows,
             color_image_.type());

}

void Dispt_pcl::create_pointcloud(){

  point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  point_cloud->resize(disparity_image_.cols * disparity_image_.rows);

}

void Dispt_pcl::convert_pointcloud(){

  pcl::PointXYZRGB temp_point;

  for(int pixel_v=0; pixel_v<disparity_image_.rows; pixel_v++){

    for(int pixel_u=0; pixel_u<disparity_image_.cols; pixel_u++){

      double disparity = (double)disparity_image_.at<uint16_t>(pixel_v, pixel_u);

      double camera_Z = baseline_fb_/(disparity/256);

      double camera_X = (pixel_u - camera_intrinsics_k_(0,2))\
          *camera_Z / camera_intrinsics_k_(0,0);

      double camera_Y = (pixel_v - camera_intrinsics_k_(1,2))\
          *camera_Z / camera_intrinsics_k_(1,1);

      temp_point.x = camera_Z;
      temp_point.y = camera_X;
      temp_point.z = -camera_Y;

      cv::Vec3b& bgr = color_image_.at<cv::Vec3b>(pixel_v, pixel_u);

      temp_point.b = bgr[0];
      temp_point.g = bgr[1];
      temp_point.r = bgr[2];

      (*point_cloud)[pixel_v*disparity_image_.cols + pixel_u] = temp_point;
    }
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Dispt_pcl::get_point_cloud(){
  return point_cloud;
}

void Dispt_pcl::viewer(){

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pcl viewer"));

//  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> fildColor(point_cloud, "z");

  //set viewer backgroud color value for rgb
  viewer->setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud);

  //add points
  //  viewer->addPointCloud<pcl::PointXYZI>(points, fildColor, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud, rgb, "sample cloud");
//  viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud, "sample cloud");

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小

  viewer->addCoordinateSystem (5.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
  }
}


}

