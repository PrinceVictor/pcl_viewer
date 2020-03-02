#include "log.h"
#include <visualizer/visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <memory>


int main(int argc, char *argv[])
{

  FLAGS_log_dir = "../log";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("---This is a point cloud viewer, supported by OpenGL---");

  //  glutInit(&argc, argv);
  //  glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);

  //  glutInitWindowPosition(0, 0);
  //  glutInitWindowSize(320, 320);

  //  glutCreateWindow("test");

  //  glutDisplayFunc(renderScene);

  //  glutReshapeFunc(changeSize);

  //  glutMainLoop();



  pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

  std::string file = argv[1];
  pcl::io::loadPCDFile(file, *points);

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pcl viewer"));

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(points, "z");

  //set viewer backgroud color value for rgb
  viewer->setBackgroundColor (0, 0, 0);

  //add points
//  viewer->addPointCloud<pcl::PointXYZI>(points, fildColor, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZI>(points, "sample cloud");

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }


  google::ShutdownGoogleLogging();

  return 0;
}
