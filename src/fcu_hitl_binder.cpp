/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/node.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_multirotor_simulator/uav_system/uav_system.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Float64Srv.h>
#include <sensor_msgs/MagneticField.h>

#include <mrs_msgs/HwApiActuatorCmd.h>
#include <mrs_msgs/HwApiControlGroupCmd.h>
#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgRateCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgCmd.h>
#include <mrs_msgs/HwApiVelocityHdgRateCmd.h>
#include <mrs_msgs/HwApiVelocityHdgCmd.h>
#include <mrs_msgs/HwApiPositionCmd.h>
#include <mrs_msgs/TrackerCommand.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/gps_conversions.h>

#include "serial_port.h"

#include <umsg.h>
#include <umsg_classes.h>

//}

namespace mrs_fcu_hitl_binder
{
    /* class FcuHiltBinder //{ */

    class FcuHiltBinder : public mrs_lib::Node
    {

    public:
        FcuHiltBinder(rclcpp::NodeOptions options);

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

        std::thread recvThread_;
        void Receiver();
        rclcpp::TimerBase::SharedPtr timer_sync_;
        void                         timerSync();

        // | ------------------------ rtf check ----------------------- |

        // | ----------------------- publishers ----------------------- |
        // mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd> ph_control_group_cmd_;

        // | -------------------------- system ------------------------ |

        // | ------------------------- methods ------------------------ |

        void calculateDelay(umsg_state_heartbeat_t heartbeat);
        uint32_t RosToFcu(rclcpp::Time &rosTime);
        rclcpp::Time FcuToRos(uint32_t &FcuTime);
        void publishPosEst(umsg_estimation_position_t &msg);

        // | ----------------------- dynamic params ------------------- |
    };

    //}

    /* FcuHiltBinder::FcuHiltBinder() //{ */

    FcuHitlBinder::FcuHitlBinder(rclcpp::NodeOptions options) : mrs_lib::Node("fcu_binder")
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

            UavSystemRos_CommonHandlers_t common_handlers;

            common_handlers.node                  = node_;
            common_handlers.uav_name              = uav_name;
            common_handlers.transform_broadcaster = tf_broadcaster_;

            uavs_.push_back(std::make_unique<UavSystemRos>(common_handlers));
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
        umsg_CRCInit();
        is_initialized_ = true;
        recvThread_ = std::thread([this]
                                  { this->Receiver(); });

        RCLCPP_INFO(node_->get_logger(), "[FcuHitlBinder]: initialized");
    }

    //}

    // | ------------------------- timers ------------------------- |

} // namespace fcu_hitl_binder

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_hitl_binders::FcuHitlBinder)
