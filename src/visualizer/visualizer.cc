#include "log.h"
#include <visualizer/visualizer.h>

#include <pcl/visualization/cloud_viewer.h>
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



