#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/node.h>
#include <mrs_lib/scope_timer.h>


#include <KDTreeVectorOfVectorsAdaptor.h>
#include <Eigen/Dense>
#include <vector>

#include <mrs_multirotor_simulator/uav_system_ros.h>
#include <mrs_multirotor_simulator/rate_counter.h>


namespace mrs_multirotor_simulator
{

/* class MultirotorSimulator //{ */

class MultirotorSimulator : public mrs_lib::Node {

public:
  MultirotorSimulator(rclcpp::NodeOptions options);

private:
  rclcpp::CallbackGroup::SharedPtr cbgrp_main_;
  rclcpp::CallbackGroup::SharedPtr cbgrp_status_;

  void initialize();

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  std::atomic<bool>        is_initialized_ = false;

  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  // | ------------------------- params ------------------------- |

  double _simulation_rate_;
  double _clock_rate_;

  rclcpp::Time sim_time_;
  rclcpp::Time last_step_time_;
  std::mutex   mutex_sim_time_;

  std::string _world_frame_name_;

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_main_;
  void                         timerMain();

  rclcpp::TimerBase::SharedPtr timer_status_;
  void                         timerStatus();

  // | ------------------------ rtf check ----------------------- |

  double       actual_rtf_ = 1.0;
  rclcpp::Time last_sim_time_status_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<rosgraph_msgs::msg::Clock>     ph_clock_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::PoseArray> ph_poses_;

  // | ------------------------- system ------------------------- |

  std::vector<std::unique_ptr<UavSystemRos>> uavs_;

  // | ------------------------- methods ------------------------ |

  void handleCollisions(void);

  void publishPoses(void);

  std::shared_ptr<mrs_lib::TransformBroadcaster> tf_broadcaster_;

  // | --------------------- dynamic params --------------------- |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;

  struct drs_params
  {
    double realtime_factor     = 1.0;
    bool   paused              = false;
    bool   collisions_enabled  = false;
    bool   collisions_crash    = false;
    double collisions_rebounce = 1;
  };

  void callbackRealtimeFactor(const double& param_value);
  void callbackPause(const bool& param_value);

  drs_params drs_params_;
  std::mutex mutex_drs_params_;
};

} // namespace mrs_multirotor_simulator