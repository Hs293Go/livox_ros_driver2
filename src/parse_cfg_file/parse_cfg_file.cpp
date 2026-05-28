//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "parse_cfg_file.h"

#include <iostream>
#include <cstdio>
#include <arpa/inet.h>
#include "simdjson.h"

namespace livox_ros {

ParseCfgFile::ParseCfgFile(const std::string& path) : path_(path) {}

bool ParseCfgFile::ParseSummaryInfo(LidarSummaryInfo& lidar_summary_info) {
  simdjson::padded_string json = simdjson::padded_string::load(path_);

  simdjson::ondemand::document doc;
  simdjson::ondemand::parser parser;
  do {
    if (parser.iterate(json).get(doc) != simdjson::error_code{}) {
      break;
    }
    simdjson::ondemand::object object;
    if (doc["lidar_summary_info"].get(object) != simdjson::error_code{}) {
      break;
    }
    uint64_t lidar_type = 0;
    if (object["lidar_type"].get(lidar_type) != simdjson::error_code{}) {
      break;
    }
    lidar_summary_info.lidar_type = static_cast<uint8_t>(lidar_type);
    return true;
  } while (false);

  std::cout << "parse lidar type failed." << std::endl;
  return false;
}

} // namespace livox_ros

