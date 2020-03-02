#ifndef __BIN2PCD_H__
#define __BIN2PCD_H__

#include <iostream>

namespace bintopcd {

using std::string;
using std::vector;

void get_filename_list(const string& path, vector<string>& filename_list);

void convert_bin2pcd(const string& in_path,
                     const string& in_file,
                     const string& out_path);

}





#endif // BIN2PCD_H
