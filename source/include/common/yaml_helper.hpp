#ifndef YAML_HELPERS_HPP
#define YAML_HELPERS_HPP

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "common/glog_sink.hpp"

struct MotionConfig {
    size_t motion_id = 0;
    std::string motion_name = "";
    size_t model_id = 0;

    // Loading configuration from a YAML node
    void load_from_yaml(const YAML::Node& config) {
        if (config["motion_id"]) motion_id = config["motion_id"].as<size_t>();
        if (config["motion_name"]) motion_name = config["motion_name"].as<std::string>();
        if (config["model_id"]) model_id = config["model_id"].as<size_t>();
    }

    // Print motion information
    void print() const {
        LOG(INFO) << "\nMotion ID: " << motion_id
                  << ", Name: " << motion_name
                  << ", Model ID: " << model_id;
    }
};

struct Motion2Config {
    size_t motion_id = 0;
    std::string motion_name = "";
    size_t model_id = 0;
    double wait_time = 0;

    // Loading configuration from a YAML node
    void load_from_yaml(const YAML::Node& config) {
        if (config["motion_id"]) motion_id = config["motion_id"].as<size_t>();
        if (config["motion_name"]) motion_name = config["motion_name"].as<std::string>();
        if (config["model_id"]) model_id = config["model_id"].as<size_t>();
        if (config["wait_time"]) wait_time = config["wait_time"].as<double>();
    }

    // Print motion information
    void print() const {
        LOG(INFO) << "\nMotion ID: " << motion_id
                  << ", Name: " << motion_name
                  << ", Model ID: " << model_id
                  << ", Wait Time: " << wait_time;
    }
};

inline void read_vector_from_yaml(const YAML::Node& node, Eigen::VectorXd& vec) {
    int size = node.size();
    vec.resize(size);
    for (int i = 0; i < size; ++i) {
        vec[i] = node[i].as<double>();
    }
}

inline void read_vector_int_from_yaml(const YAML::Node& node, Eigen::VectorXi& vec) {
    int size = node.size();
    vec.resize(size);
    for (int i = 0; i < size; ++i) {
        vec[i] = node[i].as<int>();
    }
}

inline void read_std_vector_int_from_yaml(const YAML::Node& node, std::vector<int>& vec) {
    if (!node.IsSequence()) {
        throw std::runtime_error("Expected a sequence for vector<int>");
    }

    vec.clear();
    vec.reserve(node.size());

    for (const auto& element : node) {
        vec.push_back(element.as<int>());
    }
}

inline Eigen::VectorXi get_vector_int_from_yaml(const YAML::Node& node) {
    Eigen::VectorXi vec(node.size());
    for (int i = 0; i < node.size(); ++i) {
        vec[i] = node[i].as<int>();
    }
    return vec;
}

#endif // YAML_HELPERS_HPP