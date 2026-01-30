/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/node.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_multirotor_simulator/uav_system/uav_system.hpp>

#include <sensor_msgs/sensor_msgs/msg/imu.h>
#include <sensor_msgs/sensor_msgs/msg/range.h>
#include <nav_msgs/nav_msgs/msg/odometry.h>
#include <mrs_msgs/mrs_msgs/srv/float64_srv.h>
#include <sensor_msgs/sensor_msgs/msg/magnetic_field.h>

#include <mrs_msgs/mrs_msgs/msg/hw_api_actuator_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/hw_api_control_group_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/hw_api_attitude_rate_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/hw_api_attitude_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/hw_api_acceleration_hdg_rate_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/hw_api_acceleration_hdg_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/hw_api_velocity_hdg_rate_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/hw_api_velocity_hdg_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/hw_api_position_cmd.h>
#include <mrs_msgs/mrs_msgs/msg/tracker_command.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/node.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/transform_broadcaster.h>

#include "serial_port.hpp"
#include "serial_api.hpp"

#include <umsg.h>
#include <umsg_classes.h>

#include <mrs_multirotor_simulator/uav_system_ros.h>
#include <mrs_multirotor_simulator/rate_counter.h>

//}

namespace mrs_fcu_hitl_binder
{
    /* class FcuHiltBinder //{ */

    class FcuHitlBinder : public mrs_lib::Node
    {

    public:
        FcuHitlBinder(rclcpp::NodeOptions options);

    private:
        rclcpp::CallbackGroup::SharedPtr cbgrp_main_;
        rclcpp::CallbackGroup::SharedPtr cbgrp_status_;

        void initialize();

        rclcpp::Node::SharedPtr  node_;
        rclcpp::Clock::SharedPtr clock_;
        std::atomic<bool>        is_initialized_ = false;

        std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

        // | ------------------------- params ------------------------- |

        double _clock_rate_;

        rclcpp::Time sim_time_;
        rclcpp::Time last_step_time_;
        std::mutex mutex_sim_time_;

        // | ------------------------- timers ------------------------- |

        //std::thread recvThread_;
        //rclcpp::TimerBase::SharedPtr timer_sync_;
        //void                         timerSync();

        // | ------------------------ rtf check ----------------------- |

        // | ----------------------- publishers ----------------------- |
        // mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd> ph_control_group_cmd_;

        // | -------------------------- system ------------------------ |

        std::vector<std::unique_ptr<mrs_multirotor_simulator::UavSystemRos>> uavs_;

        // | ------------------------- methods ------------------------ |

        std::shared_ptr<SerialApi> ser_api_;

        std::shared_ptr<mrs_lib::TransformBroadcaster> tf_broadcaster_;

        // | ----------------------- dynamic params ------------------- |
    };

    //}

    /* FcuHiltBinder::FcuHiltBinder() //{ */

    FcuHitlBinder::FcuHitlBinder(rclcpp::NodeOptions options) : mrs_lib::Node("fcu_binder", options)
    {
        this->initialize();
    }

    //}

    /* initialize() //{ */

    void FcuHitlBinder::initialize()
    {
        node_  = this_node_ptr();
        clock_ = node_->get_clock();

        srand(time(NULL));

        RCLCPP_INFO(node_->get_logger(), "initializing");

        cbgrp_main_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cbgrp_status_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // | ---------------- initialize param wrappers --------------- |

        mrs_lib::ParamLoader param_loader(node_, node_->get_name());

        // | ----------------------- load files ----------------------- |

        // load custom config

        std::string custom_config_path;
        param_loader.loadParam("custom_config", custom_config_path);

        if (custom_config_path != "")
        {
            RCLCPP_INFO(node_->get_logger(), "loading custom config '%s'", custom_config_path.c_str());
            param_loader.addYamlFile(custom_config_path);
        }

        // load other configs

        std::vector<std::string> config_files;
        param_loader.loadParam("simulator_configs", config_files);

        for (auto config_file : config_files)
        {
            RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
            param_loader.addYamlFile(config_file);    
        }

        // | ----------------------- load params ---------------------- |

        param_loader.loadParam("clock_rate", _clock_rate_);

        bool sim_time_from_wall_time;
        param_loader.loadParam("sim_time_from_wall_time", sim_time_from_wall_time);

        tf_broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>(node_);

        std::vector<std::string> uav_names;

        param_loader.loadParam("uav_names", uav_names);

        for (size_t i = 0; i < uav_names.size(); i++) 
        {
            std::string uav_name = uav_names.at(i);

            RCLCPP_INFO(node_->get_logger(), "initializing '%s'", uav_name.c_str());

            mrs_multirotor_simulator::UavSystemRos_CommonHandlers_t common_handlers;

            common_handlers.node                  = node_;
            common_handlers.uav_name              = uav_name;
            common_handlers.transform_broadcaster = tf_broadcaster_;

            uavs_.push_back(std::make_unique<mrs_multirotor_simulator::UavSystemRos>(common_handlers));
        }

        RCLCPP_INFO(node_->get_logger(), "all uavs initialized");

        if (!param_loader.loadedSuccessfully()) 
        {
            RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
            rclcpp::shutdown();
        }

        // | ----------------------- publishers ----------------------- |
        // ph_control_group_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>(nh_,uav_name + "/control_group_cmd",1,false);

        // | ------------------------- timers ------------------------- |

        // | ----------------------- scope timer ---------------------- |
        
        // | ----------------------- subscribers ----------------------- |

        // | ----------------------- finish init ---------------------- |
        //Initialization of serial communication done in hw_api_mrs_fcu.cpp
        //umsg_CRCInit();
        
        //ser_api_ = std::make_shared<SerialApi>(node_);
        //ser_api_->startReceiver();

        is_initialized_ = true;
        RCLCPP_INFO(node_->get_logger(), "[FcuHitlBinder]: initialized");
    }

    //}

    // | ------------------------- timers ------------------------- |

} // namespace fcu_hitl_binder

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_fcu_hitl_binder::FcuHitlBinder)
