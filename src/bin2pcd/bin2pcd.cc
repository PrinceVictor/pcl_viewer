#include <log.h>
#include <bin2pcd/bin2pcd.h>

#include <iostream>
#include <cstring>
#include <fstream>
#include <dirent.h>
#include <pcl/io/pcd_io.h>
#include <omp.h>

#include "log.h"

namespace bintopcd {

using std::string;
using std::vector;

void get_filename_list(const string& path, vector<string>& filename_list){

  DIR* dir;
  dir = opendir(path.c_str());
  struct dirent* ptr;

  while((ptr = readdir(dir)) != NULL){

    string file_name = ptr->d_name;
    auto pos = file_name.rfind(".bin");

    if(pos != string::npos){

      file_name = file_name.substr(0, pos);
      filename_list.emplace_back(file_name);
    }
  }

}

void convert_bin2pcd(const string& in_path,
                     const string& in_file,
                     const string& out_path){


  std::fstream input((in_path+ '/' + in_file + ".bin").c_str(), std::ios::in | std::ios::binary);

  if(!input.good()){

    LOG_OUTPUT("Could not read file: {}", (in_path + '/' + in_file + ".bin"));
    exit(EXIT_FAILURE);
  }

  input.seekg(0, std::ios::beg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

  int i;

  for(i=0; input.good() && !input.eof(); i++){
    pcl::PointXYZI point;
    input.read((char *)&point.x, 3*sizeof(float));
    input.read((char *)&point.intensity, sizeof(float));
    points->push_back(point);
  }

  input.close();

  pcl::PCDWriter writer;

  writer.write< pcl::PointXYZI> ((out_path + '/' + in_file + ".pcd"), *points, false);
  LOG_OUTPUT("from {} read {} points, writing to {}", in_file.c_str(), i, (out_path + '/' + in_file + ".pcd").c_str());

}

}
