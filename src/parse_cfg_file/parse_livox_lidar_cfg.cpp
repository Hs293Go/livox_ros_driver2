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

#include "parse_livox_lidar_cfg.h"
#include <iostream>

namespace livox_ros {

bool LivoxLidarConfigParser::Parse(std::vector<UserLivoxLidarConfig> &lidar_configs) {
  simdjson::padded_string json = simdjson::padded_string::load(path_);

  lidar_configs.clear();
  simdjson::ondemand::document doc;
  simdjson::ondemand::parser parser;
  parser.iterate(json).get(doc);

  do {
    if (auto error = parser.iterate(json).get(doc); error != simdjson::error_code{}) {
      std::cout << "failed to parse config json:: " << simdjson::error_message(error) << "\n";
      break;
    }
    simdjson::ondemand::array array;
    if (auto res = doc["lidar_configs"].get_array().get(array); 
        res != simdjson::error_code{} || array.count_elements() == 0) {
      std::cout << "there is no user-defined config" << std::endl;
      break;
    }
    if (!ParseUserConfigs(array, lidar_configs)) {
      std::cout << "failed to parse basic configs" << std::endl;
      break;
    }
    return true;
  } while (false);

  return false;
}

bool LivoxLidarConfigParser::ParseUserConfigs(simdjson::ondemand::array array,
                                              std::vector<UserLivoxLidarConfig> &user_configs) {
  for (auto raw_config : array) {
    simdjson::ondemand::object config;
    if (raw_config.get(config) != simdjson::error_code{}) {
      continue;
    }

    std::string_view ip_string;
    if (config["ip"].get(ip_string) != simdjson::error_code{}) {
      continue;
    }
    UserLivoxLidarConfig user_config;

    // parse user configs
    user_config.handle = IpStringToNum(std::string(ip_string));
    
    if (int64_t pcl_data_type = -1; config["pcl_data_type"].get(pcl_data_type) == simdjson::error_code{}) {
      user_config.pcl_data_type = static_cast<int8_t>(pcl_data_type);
    }
    if (int64_t pattern_mode = -1; config["pattern_mode"].get(pattern_mode) == simdjson::error_code{}) {
      user_config.pattern_mode = static_cast<int8_t>(pattern_mode);
    }
    if (int64_t blind_spot_set = -1; config["blind_spot_set"].get(blind_spot_set) == simdjson::error_code{}) {
      user_config.blind_spot_set = static_cast<int8_t>(blind_spot_set);
    }
    if (int64_t dual_emit_en = -1; config["dual_emit_en"].get(dual_emit_en) == simdjson::error_code{}) {
      user_config.dual_emit_en = static_cast<uint8_t>(dual_emit_en);
    }
    if (simdjson::ondemand::object value; config["extrinsic_parameter"].get(value) == simdjson::error_code{}) {
      if (!ParseExtrinsics(value, user_config.extrinsic_param)) {
        memset(&user_config.extrinsic_param, 0, sizeof(user_config.extrinsic_param));
        std::cout << "failed to parse extrinsic parameters, ip: "
                  << IpNumToString(user_config.handle) << std::endl;
      }
    } else {
      memset(&user_config.extrinsic_param, 0, sizeof(user_config.extrinsic_param));
    }
    user_config.set_bits = 0;
    user_config.get_bits = 0;

    user_configs.push_back(user_config);
  }

  if (0 == user_configs.size()) {
    std::cout << "no valid base configs" << std::endl;
    return false;
  }
  std::cout << "successfully parse base config, counts: "
            << user_configs.size() << std::endl;
  return true;
}

bool LivoxLidarConfigParser::ParseExtrinsics(simdjson::ondemand::object value,
                                             ExtParameter &param) {
  auto get_float_or_zero = [&value] (auto&& key) {
    if (double res = 0.0f; value[key].get(res) == simdjson::error_code{}) {
      return static_cast<float>(res);
    }
    return 0.0f;
  };

  param.roll = get_float_or_zero("roll");
  param.pitch = get_float_or_zero("pitch");
  param.yaw = get_float_or_zero("yaw");
  param.x = get_float_or_zero("x");
  param.y = get_float_or_zero("y");
  param.z = get_float_or_zero("z");

  return true;
}

} // namespace livox_ros
