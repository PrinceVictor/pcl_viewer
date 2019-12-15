#ifndef BIN2PCD_H
#define BIN2PCD_H

#include <log.h>
#include <iostream>

namespace BIN2PCD {

using std::string;
using std::vector;

void get_filename_list(const string& path, vector<string>& filename_list);

void convert_bin2pcd(const string& in_path,
                     const string& in_file,
                     const string& out_path);

}





#endif // BIN2PCD_H
