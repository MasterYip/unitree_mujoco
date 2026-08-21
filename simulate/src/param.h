#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    int camera_follow = 0;
    std::string camera_follow_body;
    std::array<double, 3> camera_lookat_offset{0.0, 0.0, 0.0};
    int camera_follow_yaw = 0;
    double camera_smoothing_tau = 0.0;
    double camera_distance = 3.0;
    double camera_azimuth = 90.0;
    double camera_elevation = -20.0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            if (cfg["camera_follow"]) camera_follow = cfg["camera_follow"].as<int>();
            if (cfg["camera_follow_body"]) camera_follow_body = cfg["camera_follow_body"].as<std::string>();
            if (cfg["camera_lookat_offset"])
            {
                const auto offset = cfg["camera_lookat_offset"].as<std::vector<double>>();
                if (offset.size() != 3) throw std::runtime_error("camera_lookat_offset must contain exactly 3 values");
                std::copy(offset.begin(), offset.end(), camera_lookat_offset.begin());
            }
            if (cfg["camera_follow_yaw"]) camera_follow_yaw = cfg["camera_follow_yaw"].as<int>();
            if (cfg["camera_smoothing_tau"]) camera_smoothing_tau = cfg["camera_smoothing_tau"].as<double>();
            if (cfg["camera_distance"]) camera_distance = cfg["camera_distance"].as<double>();
            if (cfg["camera_azimuth"]) camera_azimuth = cfg["camera_azimuth"].as<double>();
            if (cfg["camera_elevation"]) camera_elevation = cfg["camera_elevation"].as<double>();
            if (camera_smoothing_tau < 0.0) throw std::runtime_error("camera_smoothing_tau must be non-negative");
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("camera-follow", po::value<int>(&config.camera_follow)->implicit_value(1), "Follow a body with the viewer camera")
        ("camera-follow-body", po::value<std::string>(&config.camera_follow_body), "Body name to follow (default: pelvis for G1, otherwise base_link)")
        ("camera-follow-yaw", po::value<int>(&config.camera_follow_yaw)->implicit_value(1), "Rotate camera heading and offset with body yaw")
        ("camera-lookat-x", po::value<double>(&config.camera_lookat_offset[0]), "Body-relative/world-fixed look-at X offset")
        ("camera-lookat-y", po::value<double>(&config.camera_lookat_offset[1]), "Body-relative/world-fixed look-at Y offset")
        ("camera-lookat-z", po::value<double>(&config.camera_lookat_offset[2]), "Look-at Z offset")
        ("camera-smoothing-tau", po::value<double>(&config.camera_smoothing_tau), "Camera follow damping time constant in seconds")
        ("camera-distance", po::value<double>(&config.camera_distance), "Camera distance")
        ("camera-azimuth", po::value<double>(&config.camera_azimuth), "Base camera azimuth in degrees")
        ("camera-elevation", po::value<double>(&config.camera_elevation), "Camera elevation in degrees")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}
