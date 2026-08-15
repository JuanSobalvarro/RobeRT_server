#pragma once

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>


namespace robert::parser {

struct UserInfo {
    std::string username;
    std::string password;
};

struct RobotInfo {
    std::string name;
    std::string ip;
    uint16_t port;
    uint16_t timeout;
};

struct Conf {
    RobotInfo robot;
    std::vector<UserInfo> users;
};

Conf parse_conf(const std::string& path);

std::string get_conf_str(const Conf& conf);

} // namespace robert::parser
