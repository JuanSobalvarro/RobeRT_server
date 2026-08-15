#include "core/conf_parser.hpp"

#include <sstream>

namespace robert::parser {
Conf parse_conf(const std::string& path) {
    Conf conf;

    YAML::Node config_node = YAML::LoadFile(path);

    // load robot

    std::string robot_name = config_node["robot"]["name"].as<std::string>();
    std::string robot_ip = config_node["robot"]["ip"].as<std::string>();
    int robot_port = config_node["robot"]["port"].as<int>();
    int robot_timeout = config_node["robot"]["timeout"].as<int>();

    conf.robot.name = robot_name;
    conf.robot.ip = robot_ip;
    conf.robot.port = robot_port;
    conf.robot.timeout = robot_timeout;

    // load users

    YAML::Node users_node = config_node["users"];
    for (const auto& user : users_node) {
        std::string user_name = user["username"].as<std::string>();
        std::string user_password = user["password"].as<std::string>();

        conf.users.push_back({user_name, user_password});
    }

    return conf;
}

std::string get_conf_str(const Conf& conf) {
    std::stringstream ss;
    ss << "robot: " << conf.robot.name << " (" << conf.robot.ip << ":" << conf.robot.port << ")\n";
    ss << "users:\n";
    for (const auto& user : conf.users) {
        ss << "  - " << user.username << "\n";
    }
    return ss.str();
}

} // namespace robert::parser
