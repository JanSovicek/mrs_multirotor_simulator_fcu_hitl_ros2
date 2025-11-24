#include <pluginlib/class_loader.hpp>
#include <mrs_uav_hw_api/api.h>
#include <iostream>

int main(int argc, char** argv)
{
    try {
        pluginlib::ClassLoader<mrs_uav_hw_api::MrsUavHwApi> loader(
            "mrs_uav_hw_api", "mrs_uav_hw_api::MrsUavHwApi"
        );
        auto instance = loader.createUniqueInstance("mrs_uav_fcu_api::MrsUavFcuApi");
        std::cout << "Plugin loaded successfully!" << std::endl;
    } catch (const pluginlib::PluginlibException& ex) {
        std::cerr << "PluginlibException: " << ex.what() << std::endl;
    }
}