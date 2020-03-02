#include "log.h"
#include <bin2pcd/bin2pcd.h>

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <omp.h>

using std::string;

int main(int argc, char *argv[])
{

  FLAGS_log_dir = "../log";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("---This code is to convert ***.bin to ***.pcd---");

  string in_path, out_path;

  if(argc < 3){

    LOG_OUTPUT("---incomplete parameters---");
    LOG_OUTPUT("---Please run like this : ./bin_to_pcd in_path out_path---");
  }
  else if(argc == 3){

    in_path = argv[1];
    out_path = argv[2];
  }
  else{
    exit(EXIT_FAILURE);
  }

  if(access(out_path.c_str(), 0) == -1){

    mkdir(out_path.c_str(), 0777);
  }


  std::vector<string> filename_list;

  bintopcd::get_filename_list(in_path, filename_list);

//  omp_set_num_threads(4);
//#pragma omp parallel for

  for(int i=0; i<filename_list.size(); i++){

    bintopcd::convert_bin2pcd(in_path, filename_list[i], out_path);
  }

  google::ShutdownGoogleLogging();

  return 0;
}
