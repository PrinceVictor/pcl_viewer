#include <log.h>
#include <visualizer/visualizer.h>

#include <pcl/io/pcd_io.h>
#include <GL/glut.h>

#include <iostream>

void renderScene(void) {
  glClear(GL_COLOR_BUFFER_BIT);
  glBegin(GL_LINES);
  glColor3f(0, 1, 0);
  glVertex3f(0.5, 0, -5);
  glVertex3f(1.5, 1, -4);
  glEnd();

  glPointSize(5);
  glBegin(GL_POINTS);
  glColor3f(1, 0, 0);
  glVertex3f(0,0,-1);
  glEnd();
  glutSwapBuffers();
}

void changeSize(int w, int h) {

         // 防止除数即高度为0
         // (你可以设置窗口宽度为0).
         if(h == 0)
                 h = 1;

         float ratio = 1.0* w / h;

         // 单位化投影矩阵。
         glMatrixMode(GL_PROJECTION);
         glLoadIdentity();

         // 设置视口大小为增个窗口大小
         glViewport(0, 0, w, h);

         // 设置正确的投影矩阵
         gluPerspective(45,ratio,1,1000);
        //下面是设置模型视图矩阵
         glMatrixMode(GL_MODELVIEW);

         glOrtho(2, 2, 2, 2, 0, 10);
//         glLoadIdentity();
//         gluLookAt(0.0,0.0,5.0, 0.0,0.0,-1.0,0.0f,1.0f,0.0f);
}


int main(int argc, char *argv[])
{

  //  FLAGS_log_dir = "../log";
  FLAGS_log_dir = "/home/victor/mobile_robot/pcl_viewer/pointcloud_view/log";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("---This is a point cloud viewer, supported by OpenGL---");

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);

  glutInitWindowPosition(0, 0);
  glutInitWindowSize(320, 320);

  glutCreateWindow("test");

  glutDisplayFunc(renderScene);

  glutReshapeFunc(changeSize);

  glutMainLoop();



//  pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

//  std::string file = argv[1];
//  pcl::io::loadPCDFile(file, *points);

  google::ShutdownGoogleLogging();

  return 0;
}
