/* includes //{ */

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_hw_api/api.h>

#include <std_srvs/std_srvs/srv/trigger.h>

#include <mrs_modules_msgs/mrs_modules_msgs/msg/bestpos.hpp>

#include <nav_msgs/nav_msgs/msg/odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/gps_conversions.h>

// #include <mrs_msgs/mrs_msgs/msg/float64.h>

#include <geometry_msgs/geometry_msgs/msg/quaternion_stamped.h>

#include <mavros_msgs/mavros_msgs/msg/attitude_target.h>
#include <mavros_msgs/mavros_msgs/srv/command_long.h>
#include <mavros_msgs/mavros_msgs/srv/set_mode.h>
#include <mavros_msgs/mavros_msgs/msg/state.h>
#include <mavros_msgs/mavros_msgs/msg/rc_in.h>
#include <mavros_msgs/mavros_msgs/msg/altitude.h>
#include <mavros_msgs/mavros_msgs/msg/actuator_control.h>
#include <mavros_msgs/mavros_msgs/msg/gpsraw.h>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "serial_api.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <umsg.h>
#include <umsg_classes.h>
#include "umsg_sensors.h"
#include "umsg_control.h"
#include "umsg_estimation.h"
#include "umsg_offboard.h"
#include "attitude_est.hpp"

//}

/* defines //{ */

#define GRAV_CONST 9.81
#define PWM_MIDDLE 0
#define PWM_MIN -10000
#define PWM_MAX 10000
#define PWM_DEADBAND 200
#define PWM_RANGE PWM_MAX - PWM_MIN

/*Start X and Y position to be shared across hitl_binder(PC->FCU) and MrsUavFcuApi(FCU->PC)*/
static double startX, startY;

//}

namespace mrs_uav_fcu_api
{
    class hitl_binder
    {
    private:
        //std::shared_ptr<SerialApi> ser_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Clock::SharedPtr clock_;
        std::string UTM_zone;

        rclcpp::CallbackGroup::SharedPtr cbgrp_subs_;

        AttitudeEstimator attitude_estimator_;
        std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

        bool is_initialized_ = false;

        mrs_msgs::msg::HwApiCapabilities _capabilities_;

        std::string _uav_name_;
        std::string _world_frame_name_;
        std::string _body_frame_name_;

        std::string orientation_frame_id_cached_;
        std::string angular_velocity_frame_id_cached_;

        // | ----------------------- subscribers ----------------------- |
        mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu> sh_imu_;
        mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_odom_;
        mrs_lib::SubscriberHandler<sensor_msgs::msg::Range> sh_rangefinder_;
        mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_altitude_;
        mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_mag_;

        void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
        void callbackRangeFinder(const sensor_msgs::msg::Range::ConstSharedPtr msg);
        void callbackAltitude(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void callbackMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);

        void publishImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg, umsg_sensors_imu_t &msgImu, rclcpp::Time &sim_time);
        void publishMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg, umsg_sensors_mag_t &msgMag, rclcpp::Time &sim_time);
        void publishAltitude(const nav_msgs::msg::Odometry::ConstSharedPtr msg, rclcpp::Time &sim_time);
        void publishGps(const nav_msgs::msg::Odometry::ConstSharedPtr msg, rclcpp::Time &sim_time);

        // | ----------------------- publishers ----------------------- |
        mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd> ph_actuator_cmd_;

        //publishes for testing complementary filter
        mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped>      ph_orientation_com_filt_;
        mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>         ph_ang_vel_com_filt_;

        // | ----------------------- Custom publishers ----------------------- |
        void publishAttitudeEst(const umsg_estimation_attitude_t &msg);

    public:
        void initialize(const rclcpp::Node::SharedPtr parent_node, mrs_lib::ParamLoader& param_loader, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);
        bool ParseMessage(umsg_MessageToTransfer &msg);
    };

    void hitl_binder::initialize(const rclcpp::Node::SharedPtr parent_node, mrs_lib::ParamLoader& param_loader, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers)
    {
        /*Asign Node and Serial class instance*/
        node_ = parent_node;  
        clock_ = node_->get_clock();

        common_handlers_ = common_handlers;

        _uav_name_         = common_handlers->getUavName();
        _body_frame_name_  = common_handlers->getBodyFrameName();
        _world_frame_name_ = common_handlers->getWorldFrameName();

        orientation_frame_id_cached_ = _uav_name_ + "/" + _world_frame_name_;
        angular_velocity_frame_id_cached_ = _uav_name_ + "/" + _body_frame_name_;

        /*Asign callback group*/
        cbgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        /*Load position parameters*/
        double startLat, startLon;

        param_loader.loadParam("start_latitude", startLat);
        param_loader.loadParam("start_longitude", startLon);


        if (!param_loader.loadedSuccessfully()) 
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
            rclcpp::shutdown();
        }

        /*Transform latitude and longitude to xy position*/
        mrs_lib::LLtoUTM(startLat, startLon, startY, startX, UTM_zone);

        RCLCPP_INFO(node_->get_logger(),"SELECTED UTM ZONE IS : %s", UTM_zone.c_str());

        /*Prepare subscribers*/
        mrs_lib::SubscriberHandlerOptions shopts;
        shopts.node                 = node_;
        shopts.node_name            = node_->get_name();
        shopts.no_message_timeout   = mrs_lib::no_timeout;
        shopts.threadsafe           = true;
        shopts.autostart            = true;
        shopts.subscription_options.callback_group = cbgrp_subs_;

        /*Initialize subscribers*/
        sh_imu_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/hitl/imu_in", &hitl_binder::callbackIMU, this);
        sh_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/hitl/odom_in", &hitl_binder::callbackOdometry, this);
        //sh_rangefinder_ = mrs_lib::SubscriberHandler<sensor_msgs::Range>(shopts, "/~/hitl/rangefinder", &hitl_binder::callbackRangeFinder, this);
        sh_altitude_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/hitl/altitude_in", &hitl_binder::callbackAltitude, this);
        sh_mag_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>(shopts, "~/hitl/magnetometer_in", &hitl_binder::callbackMag, this);

        /*Initialize publishers*/
        ph_actuator_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd>(node_, "~/hitl/actuators_cmd_out");
        ph_orientation_com_filt_ = mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped>(node_, "~/hitl/orientation_com_filt_out");
        ph_ang_vel_com_filt_ = mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>(node_, "~/hitl/ang_vel_com_filt_out");

        RCLCPP_INFO(node_->get_logger(),"Subscribers and Publishers initialized");

        /*Init attitude estimator*/
        attitude_estimator_.Init();

        is_initialized_ = true;
    };

    // | ------------------------ Testing publishers ----------------------- |

    void hitl_binder::publishAttitudeEst(const umsg_estimation_attitude_t &msg)
    {
        if (!is_initialized_)
        {
            return;
        }
        
        /*----Publish orientation ----*/
            geometry_msgs::msg::QuaternionStamped orientation;
            // Direct 64-bit integer scale to avoid implicit floating-point conversions
            orientation.header.stamp = rclcpp::Time(static_cast<int64_t>(msg.timestamp) * 1000LL, RCL_ROS_TIME);

            // Zero-allocation assignment using the pre-cached frame identifier
            orientation.header.frame_id = orientation_frame_id_cached_;

            // Mathematical Safety Guard: Enforce normalization to protect downstream estimators
            Eigen::Quaternion<double> q_eig(
                static_cast<double>(msg.w),
                static_cast<double>(msg.x),
                static_cast<double>(msg.y),
                static_cast<double>(msg.z)
            );
            q_eig.normalize(); 

            // Clean assignment to target ROS message fields
            orientation.quaternion.x = q_eig.x();
            orientation.quaternion.y = q_eig.y();
            orientation.quaternion.z = q_eig.z();
            orientation.quaternion.w = q_eig.w();

            ph_orientation_com_filt_.publish(orientation);
            //common_handlers_->publishers.publishOrientation(orientation);

        /*---- Publish angular velocity ----*/


            geometry_msgs::msg::Vector3Stamped angular_velocity;

            angular_velocity.header.stamp = rclcpp::Time(static_cast<int64_t>(msg.timestamp) * 1000LL, RCL_ROS_TIME);
            angular_velocity.header.frame_id = angular_velocity_frame_id_cached_;
            
            geometry_msgs::msg::Vector3 v;
            v.x = static_cast<double>(msg.att_rate[0]);
            v.y = static_cast<double>(msg.att_rate[1]);
            v.z = static_cast<double>(msg.att_rate[2]);
            angular_velocity.vector = v;

            ph_ang_vel_com_filt_.publish(angular_velocity);
      };

    /*| ------------------------- Publishers ------------------------- |*/

    // PublishImu//{
    void hitl_binder::publishImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg, umsg_sensors_imu_t &msgImu, rclcpp::Time &sim_time)
    { /*//{*/
        static double index = 0;

        /*Set payload*/
        msgImu.accel[0] = static_cast<float>(msg->linear_acceleration.x / GRAV_CONST);
        msgImu.accel[1] = static_cast<float>(msg->linear_acceleration.y / GRAV_CONST);
        msgImu.accel[2] = static_cast<float>(msg->linear_acceleration.z / GRAV_CONST);

        msgImu.gyro[0] = static_cast<float>(msg->angular_velocity.x);
        msgImu.gyro[1] = static_cast<float>(msg->angular_velocity.y);
        msgImu.gyro[2] = static_cast<float>(msg->angular_velocity.z);
        msgImu.temperature = index;
        msgImu.timestamp = static_cast<uint64_t>(sim_time.nanoseconds()*1e-3); // Convert to microseconds

    } /*//}*/ /*//}*/

    void hitl_binder::publishMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg, umsg_sensors_mag_t &msgMag, rclcpp::Time &sim_time)
    {
        /*Set payload*/
        msgMag.mag[0] = static_cast<float>(msg->magnetic_field.x);
        msgMag.mag[1] = static_cast<float>(msg->magnetic_field.y);
        msgMag.mag[2] = static_cast<float>(msg->magnetic_field.z);
        msgMag.timestamp = static_cast<uint64_t>(sim_time.nanoseconds()*1e-3); // Convert to microseconds

    }

    void hitl_binder::publishAltitude(const nav_msgs::msg::Odometry::ConstSharedPtr msg, rclcpp::Time &sim_time)
    {
        /*Set header*/
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_ALTIMETER;

        //Declare Magnetometer message
        umsg_sensors_altimeter_t msgAlt;

        /*Set payload*/
        msgAlt.altitude = static_cast<float>(msg->pose.pose.position.z);
        //msgAlt.timestamp = ser_->RosToFcu(sim_time);

        /*Serialize message*/
        uint32_t payload_len = umsg_sensors_altimeter_serialize(&msgAlt, out.s.payload);

        /*Set message length and CRC*/
        out.s.len = payload_len + UMSG_HEADER_SIZE + UMSG_CRC_SIZE;
        out.raw[out.s.len - UMSG_CRC_SIZE] = umsg_calcCRC(out.raw, out.s.len - UMSG_CRC_SIZE);

        /*Send message*/
        //ser_->sendPacket(out);
    }

    void hitl_binder::publishGps(const nav_msgs::msg::Odometry::ConstSharedPtr msg, rclcpp::Time &sim_time)
    {
        /*Set header*/
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_GPS;

        //Declare GPS message
        umsg_sensors_gps_t msgGps;

        /*Set payload*/
        //msgGps.timestamp = ser_->RosToFcu(sim_time);
        //RCLCPP_INFO(node_->get_logger(),"[HITL BINDER] GPS ros time: %ld ns, GPS FCU time: %u ms", sim_time.nanoseconds(), out.s.sensors.gps.timestamp);
        msgGps.fixType = FIX_3D;
        msgGps.hELPS = msg->pose.pose.position.z;
        msgGps.hMSL = msg->pose.pose.position.z;
        msgGps.reserved = 0;
        msgGps.numSV = 20;

        /*Convert local cartesian to global GNSS coordinates*/
        double UTMNorth, UTMEast;
        UTMEast = startX + msg->pose.pose.position.x;
        UTMNorth = startY + msg->pose.pose.position.y;
        double lat, lon;
        mrs_lib::UTMtoLL(UTMNorth, UTMEast, UTM_zone, lat, lon);

        /*Set payload*/
        msgGps.lat = lat;
        msgGps.lon = lon;

        msgGps.CRCValid = 1;
        msgGps.DataValid = 1;
        msgGps.gnssFixOk = 1;

        //Eigen::Vector3d vel_body = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        //Eigen::Quaternion<double> q = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        //Eigen::Matrix3d R = q.toRotationMatrix();
        //Eigen::Vector3d vel_world = R * vel_body;

        // Note: GPS velocity is typically in the body frame, but here we are sending world frame velocity to replicate GPS velocity noise, which is independent of the drone's orientation
        // GPS velocity format is NED (north, east, down), hence the negation of Z component
        msgGps.vel[0] = msg->twist.twist.linear.y;
        msgGps.vel[1] = msg->twist.twist.linear.x;
        msgGps.vel[2] = -msg->twist.twist.linear.z;

        /*Serialize message*/
        uint32_t payload_len = umsg_sensors_gps_serialize(&msgGps, out.s.payload);

        /*Set message length and CRC*/
        out.s.len = payload_len + UMSG_HEADER_SIZE + UMSG_CRC_SIZE;
        out.raw[out.s.len - UMSG_CRC_SIZE] = umsg_calcCRC(out.raw, out.s.len - UMSG_CRC_SIZE);

        /*Send message*/
        //ser_->sendPacket(out);
    }

    /*| ------------------------- callbacks ------------------------- |*/

    void hitl_binder::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {

        /*Get time from msg and publish gps*/
        rclcpp::Time sim_time = msg->header.stamp;
        //publishGps(msg, sim_time);
        RCLCPP_INFO_ONCE(node_->get_logger(), "[HITLBinder]: GPS CALLBACK called");
    }

    void hitl_binder::callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {

        /*Extract time from msg*/
        rclcpp::Time sim_time = msg->header.stamp;

        umsg_sensors_imu_t msgImu;

        /*Publish Imu*/
        publishImu(msg, msgImu, sim_time);

        /*Call complementary filter update */
        attitude_estimator_.UpdateImu(msgImu);

        /*Get and publish attitude*/
        bool is_attitude_valid;
        Eigen::Quaternion<float> q = attitude_estimator_.GetEstimation(&is_attitude_valid);

        umsg_estimation_attitude_t msgAtt;
        msgAtt.timestamp = msgImu.timestamp;
        msgAtt.att_rate[0] = msgImu.gyro[0];
        msgAtt.att_rate[1] = msgImu.gyro[1];
        msgAtt.att_rate[2] = msgImu.gyro[2];
        msgAtt.w = q.w();
        msgAtt.x = q.x();
        msgAtt.y = q.y();
        msgAtt.z = q.z();

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[HITLBinder]: Attitude time: %lu (micro s), Attitude estimate: w=%f, x=%f, y=%f, z=%f", msgAtt.timestamp, msgAtt.w, msgAtt.x, msgAtt.y, msgAtt.z);

        /*Publish attitude*/
        if(is_attitude_valid)
        { 
            RCLCPP_INFO_ONCE(node_->get_logger(),"[HITLBinder]: Attitude is valid, publishing estimate");
            publishAttitudeEst(msgAtt);
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[HITLBinder]: IMU CALLBACK called");

        //RCLCPP_INFO(node_->get_logger(),"[FcuBinder]: IMU Duration %d",diff_to_now.nanoseconds());
    }

    void hitl_binder::callbackRangeFinder(const sensor_msgs::msg::Range::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        RCLCPP_WARN_ONCE(node_->get_logger(),"rangefinder callback not yet implemented");
    }

    void hitl_binder::callbackAltitude(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /*Extract time from msg*/
        rclcpp::Time sim_time = msg->header.stamp;
        /*Publish Altitude*/
        //publishAltitude(msg, sim_time);
        RCLCPP_INFO_ONCE(node_->get_logger(), "[HITLBinder]: Altitude CALLBACK called");

        //RCLCPP_INFO(node_->get_logger(), "[FcuBinder]: IMU Duration %d",diff_to_now.nanoseconds());
    }

    void hitl_binder::callbackMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
    {
        /*Extract time from msg*/
        rclcpp::Time sim_time = msg->header.stamp;
        /*Publish magnetometer*/
        umsg_sensors_mag_t msgMag;
        publishMag(msg, msgMag, sim_time);
        /*Swap mag direction*/
        //float mag_x = msgMag.mag[0];
        //msgMag.mag[0] = msgMag.mag[1];
        //msgMag.mag[1] = mag_x;

        /*Update attitude estimator*/
        attitude_estimator_.UpdateMag(msgMag);

        RCLCPP_INFO_ONCE(node_->get_logger(),"[FcuBinder]: mag CALLBACK called");

        // RCLCPP_INFO(node_->get_logger(),"[FcuBinder]: IMU Duration %d",diff_to_now.nanoseconds());
    }

    bool hitl_binder::ParseMessage(umsg_MessageToTransfer &msg)
    {
        uint8_t msg_class = msg.s.msg_class;
        uint8_t msg_type = msg.s.msg_type;

        bool parsed = true;

        switch (msg_class)
        {
            case UMSG_CONTROL:
            {
                //RCLCPP_INFO(node_->get_logger(), "received DSHOT MSG");

                switch (msg_type)
                {    
                    case CONTROL_DSHOTMESSAGE:
                    {
                        umsg_control_DshotMessage_t msgDshot;
                        bool bSuccess = umsg_control_DshotMessage_deserialize(&msgDshot, msg.s.payload, msg.s.len - UMSG_HEADER_SIZE-UMSG_CRC_SIZE);

                        if(true == bSuccess)
                        {
                            mrs_msgs::msg::HwApiActuatorCmd cmd;
                            //cmd.stamp = ser_->FcuToRos(msgDshot.timestamp);

                            for (size_t i = 0; i < 4; i++)
                            {
                                cmd.motors.push_back(static_cast<float>(msgDshot.channels[i] - 48) / 2048.);
                            }
                            ph_actuator_cmd_.publish(cmd);
                        }
                        else 
                        {
                            //Drop the message
                            RCLCPP_ERROR(node_->get_logger(),"[MrsUavFcuApi]: DShot message deserialization failed");
                        }
                    }
                    break;

                    default:
                        parsed = false;
                        break;
                } 
            }
            break;

            default:
                parsed = false;
                break;
        }
        return parsed;
    }

    /**###############################################################
    * --------------------------------------------------------------
    * |                   controller's interface                   |
    * --------------------------------------------------------------
    * ################################################################*/

    /* class MrsUavFcuApi //{ */

    class MrsUavFcuApi : public mrs_uav_hw_api::MrsUavHwApi {

    public:
    ~MrsUavFcuApi() {};

    void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

    void destroy();

    rclcpp::Node::SharedPtr  node_;
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::CallbackGroup::SharedPtr cbgrp_subs_;
    rclcpp::CallbackGroup::SharedPtr cbgrp_sc_;
    rclcpp::CallbackGroup::SharedPtr cbgrp_timers_;

    // | ------------------------- params ------------------------- |

    mrs_msgs::msg::HwApiCapabilities _capabilities_;

    bool _publish_orientation_com_filt_;
    bool _publish_ang_vel_com_filt_;

    bool _orietation_output_is_com_filt_estimate_;

    bool _feedforward_enabled_;

    double      _utm_x_;
    double      _utm_y_;
    std::string _utm_zone_;
    double      _amsl_;

    std::string _uav_name_;
    std::string _world_frame_name_;
    std::string _body_frame_name_;

    double _input_timeout_;

    // | --------------------- status methods --------------------- |

    mrs_msgs::msg::HwApiStatus       getStatus();
    mrs_msgs::msg::HwApiCapabilities getCapabilities();

    // | --------------------- topic callbacks -------------------- |

    bool callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
    bool callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
    bool callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
    bool callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
    bool callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
    bool callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
    bool callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
    bool callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
    bool callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);
    void callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

    void callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void callbackImuNoise(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void callbackMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
    void callbackRangefinder(const sensor_msgs::msg::Range::ConstSharedPtr msg);

    // | -------------------- service callbacks ------------------- |

    std::tuple<bool, std::string> callbackArming(const bool &request);
    std::tuple<bool, std::string> callbackOffboard(void);

    private:
    bool is_initialized_ = false;

    std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

    rclcpp::Time last_cmd_time_;
    std::mutex   mutex_last_cmd_time_;

    hitl_binder hitl_binder_;

    // | ----------------------- subscribers ---------------------- |

    mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>         sh_odom_;
    mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>           sh_imu_;
    mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>           sh_imu_noise_;
    mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_mag_;
    mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>         sh_range_;

    // | ----------------------- publishers ----------------------- |

    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd>            ph_actuators_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiControlGroupCmd>        ph_control_group_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>        ph_attitude_rate_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeCmd>            ph_attitude_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd> ph_acceleration_hdg_rate_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgCmd>     ph_acceleration_hdg_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>     ph_velocity_hdg_rate_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgCmd>         ph_velocity_hdg_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiPositionCmd>            ph_position_cmd_;
    mrs_lib::PublisherHandler<mrs_msgs::msg::TrackerCommand>              ph_tracker_cmd_;

    mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped>      ph_orientation_com_filt_;
    mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>         ph_ang_vel_com_filt_;

    // | ------------------------- timers ------------------------- |

    std::shared_ptr<TimerType> timer_main_;

    void timerMain();

    // | ------------------------ variables ----------------------- |

    std::atomic<bool> offboard_  = false;
    std::string       mode_      = "NORMAL";
    std::atomic<bool> armed_     = false;
    std::atomic<bool> connected_ = false;
    std::mutex        mutex_status_;

    std::mutex attitude_est_mutex_;
    AttitudeEstimator attitude_estimator_;
    std::mutex attitude_msg_mutex_;
    umsg_estimation_attitude_t latest_attitude_;
    bool is_attitude_valid_ = false;

    // | ------------------------- methods ------------------------ |

    void translateImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg, umsg_sensors_imu_t &msgImu, rclcpp::Time &sim_time);
    
    void translateMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg, umsg_sensors_mag_t &msgMag, rclcpp::Time &sim_time);

    void publishAttitudeEst(const umsg_estimation_attitude_t &msg);

    void publishBatteryState(void);

    void publishRC(void);

    void timeoutInputs(void);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MrsUavFcuApi::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {

  node_  = node;
  clock_ = node_->get_clock();

  cbgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  common_handlers_ = common_handlers;

  _capabilities_.api_name = "MrsSimulator";

  _uav_name_         = common_handlers->getUavName();
  _body_frame_name_  = common_handlers->getBodyFrameName();
  _world_frame_name_ = common_handlers->getWorldFrameName();

  last_cmd_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader local_param_loader(node_, "MultirotorSimulatorHwApi");

  std::string custom_config_path;

  common_handlers_->main_param_loader->loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    local_param_loader.addYamlFile(custom_config_path);
  }

  std::vector<std::string> config_files;
  common_handlers_->main_param_loader->loadParamReusable("configs", config_files);

  if (!common_handlers_->main_param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
    local_param_loader.addYamlFile(config_file);
  }

  local_param_loader.loadParam("input_timeout", _input_timeout_);

  local_param_loader.loadParam("gnss/utm_x", _utm_x_);
  local_param_loader.loadParam("gnss/utm_y", _utm_y_);
  local_param_loader.loadParam("gnss/utm_zone", _utm_zone_);
  local_param_loader.loadParam("gnss/amsl", _amsl_);

  local_param_loader.loadParam("input_mode/actuators", (bool &)_capabilities_.accepts_actuator_cmd);
  local_param_loader.loadParam("input_mode/control_group", (bool &)_capabilities_.accepts_control_group_cmd);
  local_param_loader.loadParam("input_mode/attitude_rate", (bool &)_capabilities_.accepts_attitude_rate_cmd);
  local_param_loader.loadParam("input_mode/attitude", (bool &)_capabilities_.accepts_attitude_cmd);
  local_param_loader.loadParam("input_mode/acceleration_hdg_rate", (bool &)_capabilities_.accepts_acceleration_hdg_rate_cmd);
  local_param_loader.loadParam("input_mode/acceleration_hdg", (bool &)_capabilities_.accepts_acceleration_hdg_cmd);
  local_param_loader.loadParam("input_mode/velocity_hdg_rate", (bool &)_capabilities_.accepts_velocity_hdg_rate_cmd);
  local_param_loader.loadParam("input_mode/velocity_hdg", (bool &)_capabilities_.accepts_velocity_hdg_cmd);
  local_param_loader.loadParam("input_mode/position", (bool &)_capabilities_.accepts_position_cmd);
  local_param_loader.loadParam("input_mode/feedforward", _feedforward_enabled_);

  local_param_loader.loadParam("outputs/distance_sensor", (bool &)_capabilities_.produces_distance_sensor);
  local_param_loader.loadParam("outputs/gnss", (bool &)_capabilities_.produces_gnss);
  local_param_loader.loadParam("outputs/rtk", (bool &)_capabilities_.produces_rtk);
  local_param_loader.loadParam("outputs/imu", (bool &)_capabilities_.produces_imu);
  local_param_loader.loadParam("outputs/altitude", (bool &)_capabilities_.produces_altitude);
  local_param_loader.loadParam("outputs/magnetometer_heading", (bool &)_capabilities_.produces_magnetometer_heading);
  local_param_loader.loadParam("outputs/rc_channels", (bool &)_capabilities_.produces_rc_channels);
  local_param_loader.loadParam("outputs/battery_state", (bool &)_capabilities_.produces_battery_state);
  local_param_loader.loadParam("outputs/position", (bool &)_capabilities_.produces_position);
  local_param_loader.loadParam("outputs/orientation", (bool &)_capabilities_.produces_orientation);
  local_param_loader.loadParam("outputs/velocity", (bool &)_capabilities_.produces_velocity);
  local_param_loader.loadParam("outputs/angular_velocity", (bool &)_capabilities_.produces_angular_velocity);
  local_param_loader.loadParam("outputs/odometry", (bool &)_capabilities_.produces_odometry);
  local_param_loader.loadParam("outputs/ground_truth", (bool &)_capabilities_.produces_ground_truth);

  local_param_loader.loadParam("outputs/orientation_com_filt", (bool&)_publish_orientation_com_filt_);
  local_param_loader.loadParam("outputs/ang_vel_com_filt", (bool&)_publish_ang_vel_com_filt_);

  local_param_loader.loadParam("outputs/orientation_com_filt_estimate", (bool&)_orietation_output_is_com_filt_estimate_);

  _capabilities_.produces_magnetic_field = true;

  if (!local_param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = "MultirotorSimulatorHwApi";
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbgrp_subs_;

  sh_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/simulator_odom_in", &MrsUavFcuApi::callbackOdom, this);

  sh_imu_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/simulator_imu_in", &MrsUavFcuApi::callbackImu, this);

  sh_imu_noise_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/simulator_imu_noise_in", &MrsUavFcuApi::callbackImuNoise, this);

  sh_mag_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>(shopts, "~/simulator_magnetometer_in", &MrsUavFcuApi::callbackMag, this);

  sh_range_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>(shopts, "~/simulator_rangefinder_in", &MrsUavFcuApi::callbackRangefinder, this);

  // | ----------------------- publishers ----------------------- |

  if (_capabilities_.accepts_actuator_cmd) {
    ph_actuators_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd>(node_, "~/simulator_actuators_cmd_out");
  }

  if (_capabilities_.accepts_control_group_cmd) {
    ph_control_group_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiControlGroupCmd>(node_, "~/simulator_control_group_cmd_out");
  }

  if (_capabilities_.accepts_attitude_rate_cmd) {
    ph_attitude_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>(node_, "~/simulator_attitude_rate_cmd_out");
  }

  if (_capabilities_.accepts_attitude_cmd) {
    ph_attitude_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeCmd>(node_, "~/simulator_attitude_cmd_out");
  }

  if (_capabilities_.accepts_acceleration_hdg_rate_cmd) {
    ph_acceleration_hdg_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd>(node_, "~/simulator_acceleration_hdg_rate_cmd_out");
  }

  if (_capabilities_.accepts_acceleration_hdg_cmd) {
    ph_acceleration_hdg_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgCmd>(node_, "~/simulator_acceleration_hdg_cmd_out");
  }

  if (_capabilities_.accepts_velocity_hdg_rate_cmd) {
    ph_velocity_hdg_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>(node_, "~/simulator_velocity_hdg_rate_cmd_out");
  }

  if (_capabilities_.accepts_velocity_hdg_cmd) {
    ph_velocity_hdg_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgCmd>(node_, "~/simulator_velocity_hdg_cmd_out");
  }

  if (_capabilities_.accepts_position_cmd) {
    ph_position_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiPositionCmd>(node_, "~/simulator_position_cmd_out");
  }

  if (_feedforward_enabled_) {
    ph_tracker_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::TrackerCommand>(node_, "~/simulator_tracker_cmd_out");
  }

  if (_publish_orientation_com_filt_)
  {
    ph_orientation_com_filt_ = mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped>(node_, "~/orientation_com_filt_out");
  }
  
  if (_publish_ang_vel_com_filt_)
  {
    ph_ang_vel_com_filt_ = mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>(node_, "~/ang_vel_com_filt_out");
  }
  

        RCLCPP_INFO(node_->get_logger(),"Subscribers and Publishers initialized");

  // | ------------------------- timers ------------------------- |

  {
    std::function<void()> callback_fcn = std::bind(&MrsUavFcuApi::timerMain, this);

    mrs_lib::TimerHandlerOptions opts;

    opts.node           = node_;
    opts.autostart      = true;
    opts.callback_group = cbgrp_timers_;

    timer_main_ = std::make_shared<TimerType>(opts, rclcpp::Rate(10.0, clock_), callback_fcn);
  }

  // | ----------------------- finish init ---------------------- |

  /*Init HITL binder*/
  //hitl_binder_.initialize(node_, local_param_loader, common_handlers);

  /*Init attitude estimator*/
  attitude_estimator_.Init();

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* destroy() //{ */

void MrsUavFcuApi::destroy() {

  timer_main_->stop();
}

//}

/* getStatus() //{ */

mrs_msgs::msg::HwApiStatus MrsUavFcuApi::getStatus() {

  mrs_msgs::msg::HwApiStatus status;

  status.stamp = clock_->now();

  bool has_odom = sh_odom_.hasMsg();

  {
    std::scoped_lock lock(mutex_status_);

    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = has_odom;
    status.mode      = mode_;
  }

  return status;
}

//}

/* getCapabilities() //{ */

mrs_msgs::msg::HwApiCapabilities MrsUavFcuApi::getCapabilities() {

  _capabilities_.stamp = clock_->now();

  return _capabilities_;
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> MrsUavFcuApi::callbackArming([[maybe_unused]] const bool &request) {

  std::stringstream ss;

  if (request) {

    armed_ = true;

    ss << "armed";
    RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(true, ss.str());

  } else {

    armed_ = false;

    ss << "disarmed";
    RCLCPP_INFO_STREAM(node_->get_logger(), "" << ss.str());
    return std::tuple(true, ss.str());
  }
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> MrsUavFcuApi::callbackOffboard(void) {

  std::stringstream ss;

  if (!armed_) {
    ss << "Cannot switch to offboard, not armed.";
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
    return {false, ss.str()};
  }

  auto last_cmd_time = mrs_lib::get_mutexed(mutex_last_cmd_time_, last_cmd_time_);

  if ((clock_->now() - last_cmd_time).seconds() > _input_timeout_) {
    ss << "Cannot switch to offboard, missing control input.";
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
    return {false, ss.str()};
  }

  offboard_ = true;
  mode_     = "OFFBOARD";

  ss << "Offboard set";
  RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
  return {true, ss.str()};
}

//}

// | --------------------- input callbacks -------------------- |

/* callbackActuatorCmd() //{ */

bool MrsUavFcuApi::callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_actuator_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting actuators cmd");

  if (offboard_) {
    ph_actuators_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackControlGroupCmd() //{ */

bool MrsUavFcuApi::callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_control_group_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting control group cmd");

  if (offboard_) {
    ph_control_group_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool MrsUavFcuApi::callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_attitude_rate_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude rate cmd");

  if (offboard_) {
    ph_attitude_rate_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool MrsUavFcuApi::callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_attitude_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude cmd");

  if (offboard_) {
    ph_attitude_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool MrsUavFcuApi::callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_acceleration_hdg_rate_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg rate cmd");

  if (offboard_) {
    ph_acceleration_hdg_rate_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool MrsUavFcuApi::callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_acceleration_hdg_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg cmd");

  if (offboard_) {
    ph_acceleration_hdg_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool MrsUavFcuApi::callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_velocity_hdg_rate_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg rate cmd");

  if (offboard_) {
    ph_velocity_hdg_rate_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool MrsUavFcuApi::callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_velocity_hdg_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg cmd");

  if (offboard_) {
    ph_velocity_hdg_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackPositionCmd() //{ */

bool MrsUavFcuApi::callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_position_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting position cmd");

  if (offboard_) {
    ph_position_cmd_.publish(*msg);
  }

  {
    std::scoped_lock lock(mutex_last_cmd_time_);

    last_cmd_time_ = clock_->now();
  }

  return true;
}

//}

/* callbackTrackerCmd() //{ */

void MrsUavFcuApi::callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting tracker cmd");

  if (offboard_) {
    ph_tracker_cmd_.publish(*msg);
  }
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackOdom() */

void MrsUavFcuApi::callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting simulator odometry");

  auto odom = msg;

  umsg_estimation_attitude_t attitude_msg;
  bool is_attitude_valid = false;

  {
    std::scoped_lock lock(mutex_status_);

    connected_ = true;
  }

  // | ----------------- publish the diagnostics ---------------- |

  mrs_msgs::msg::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp     = clock_->now();
    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  common_handlers_->publishers.publishStatus(status);

  // | -------------------- publish position -------------------- |

  if (_capabilities_.produces_position) {

    geometry_msgs::msg::PointStamped position;

    position.header.stamp    = odom->header.stamp;
    position.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    position.point           = odom->pose.pose.position;

    common_handlers_->publishers.publishPosition(position);
  }

  // | ------------------- publish orientation ------------------ |

  if (_capabilities_.produces_orientation) {
  
    geometry_msgs::msg::QuaternionStamped orientation;
  
    orientation.header.stamp    = odom->header.stamp;
    orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_;

    if(false == _orietation_output_is_com_filt_estimate_)
    {
      orientation.quaternion = odom->pose.pose.orientation;
    }
    else
    {
      {
        std::lock_guard<std::mutex> lock(attitude_msg_mutex_);
        attitude_msg = latest_attitude_;
        is_attitude_valid = is_attitude_valid_;
      }

      if(is_attitude_valid)
      {

        orientation.quaternion.set__w(static_cast<double>(attitude_msg.w));
        orientation.quaternion.set__x(static_cast<double>(attitude_msg.x));
        orientation.quaternion.set__y(static_cast<double>(attitude_msg.y));
        orientation.quaternion.set__z(static_cast<double>(attitude_msg.z));                          
      }
      else 
      {
        orientation.quaternion.set__w(static_cast<double>(1.0));
        orientation.quaternion.set__x(static_cast<double>(0.0));
        orientation.quaternion.set__y(static_cast<double>(0.0));
        orientation.quaternion.set__z(static_cast<double>(0.0)); 
      }
    }

    common_handlers_->publishers.publishOrientation(orientation);
  }

  // | -------------------- publish velocity -------------------- |

  if (_capabilities_.produces_velocity) {

    geometry_msgs::msg::Vector3Stamped velocity;

    velocity.header.stamp    = odom->header.stamp;
    velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    velocity.vector          = odom->twist.twist.linear;

    common_handlers_->publishers.publishVelocity(velocity);
  }

  // | ---------------- publish angular velocity ---------------- |

  if (_capabilities_.produces_angular_velocity) {

    geometry_msgs::msg::Vector3Stamped angular_velocity;

    angular_velocity.header.stamp    = odom->header.stamp;
    angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    angular_velocity.vector          = odom->twist.twist.angular;

    common_handlers_->publishers.publishAngularVelocity(angular_velocity);
  }

  // | -------------------- publish odometry -------------------- |

  if (_capabilities_.produces_odometry) {

    nav_msgs::msg::Odometry odometry = *odom;
    if(_orietation_output_is_com_filt_estimate_ == true)
    {
      if(is_attitude_valid)
      {
        odometry.pose.pose.orientation.set__w(static_cast<double>(attitude_msg.w));
        odometry.pose.pose.orientation.set__x(static_cast<double>(attitude_msg.x));
        odometry.pose.pose.orientation.set__y(static_cast<double>(attitude_msg.y));
        odometry.pose.pose.orientation.set__z(static_cast<double>(attitude_msg.z));
      }
      else 
      {
        odometry.pose.pose.orientation.set__w(static_cast<double>(1.0));
        odometry.pose.pose.orientation.set__x(static_cast<double>(0.0));
        odometry.pose.pose.orientation.set__y(static_cast<double>(0.0));
        odometry.pose.pose.orientation.set__z(static_cast<double>(0.0));
      }
    }
    common_handlers_->publishers.publishOdometry(odometry);
  }

  // | ------------------ publish ground truth ------------------ |

  if (_capabilities_.produces_ground_truth) {
    common_handlers_->publishers.publishGroundTruth(*odom);
  }

  // | ---------------------- publish gnss ---------------------- |

  if (_capabilities_.produces_gnss) {

    double lat;
    double lon;

    mrs_lib::UTMtoLL(odom->pose.pose.position.y + _utm_y_, odom->pose.pose.position.x + _utm_x_, _utm_zone_, lat, lon);

    sensor_msgs::msg::NavSatFix gnss;

    gnss.header.stamp    = odom->header.stamp;
    gnss.header.frame_id = _uav_name_ + "/" + _body_frame_name_;

    gnss.latitude  = lat;
    gnss.longitude = lon;
    gnss.altitude  = odom->pose.pose.position.z + _amsl_;

    common_handlers_->publishers.publishGNSS(gnss);
  }

  // | ----------------------- publish rtk ---------------------- |

  if (_capabilities_.produces_rtk) {

    double lat;
    double lon;

    mrs_lib::UTMtoLL(odom->pose.pose.position.y + _utm_y_, odom->pose.pose.position.x + _utm_x_, _utm_zone_, lat, lon);

    mrs_msgs::msg::RtkGps rtk;

    rtk.header.stamp = odom->header.stamp;

    rtk.gps.latitude  = lat;
    rtk.gps.longitude = lon;
    rtk.gps.altitude  = odom->pose.pose.position.z + _amsl_;

    rtk.fix_type.fix_type = mrs_msgs::msg::RtkFixType::RTK_FIX;

    common_handlers_->publishers.publishRTK(rtk);
  }

  // | ------------------ publish amsl altitude ----------------- |

  if (_capabilities_.produces_altitude) {

    mrs_msgs::msg::HwApiAltitude altitude;

    altitude.stamp = odom->header.stamp;

    altitude.amsl = odom->pose.pose.position.z + _amsl_;

    common_handlers_->publishers.publishAltitude(altitude);
  }

  // | --------------------- publish heading -------------------- |

  if (_capabilities_.produces_magnetometer_heading) {

    double heading = 0;
    try {
      heading = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
      RCLCPP_WARN(node_->get_logger(), "exception caught: '%s'", e.what());
    }

    mrs_msgs::msg::Float64Stamped hdg;

    hdg.header.stamp = clock_->now();
    hdg.value        = heading;

    common_handlers_->publishers.publishMagnetometerHeading(hdg);
  }
}

//}

/* callbackImu() //{ */

void MrsUavFcuApi::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "IMU CALLBACK called");

  if (_capabilities_.produces_imu) {

    common_handlers_->publishers.publishIMU(*msg);
  }
}
//}

/* callbackImuNoise() //{ */

void MrsUavFcuApi::callbackImuNoise(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  /*Extract time from msg*/
  rclcpp::Time sim_time = msg->header.stamp;

  umsg_sensors_imu_t msgImu;

  /*Publish Imu*/
  translateImu(msg, msgImu, sim_time);

  /*Call complementary filter update */
  bool is_attitude_valid;
  Eigen::Quaternion<float> q;
  {
    std::lock_guard<std::mutex> lock(attitude_est_mutex_);
    attitude_estimator_.UpdateImu(msgImu);
    q = attitude_estimator_.GetEstimation(&is_attitude_valid);
  }

  /*Fill umsg message*/
  umsg_estimation_attitude_t msgAtt;
  msgAtt.timestamp = msgImu.timestamp;
  msgAtt.att_rate[0] = msgImu.gyro[0];
  msgAtt.att_rate[1] = msgImu.gyro[1];
  msgAtt.att_rate[2] = msgImu.gyro[2];
  msgAtt.w = q.w();
  msgAtt.x = q.x();
  msgAtt.y = q.y();
  msgAtt.z = q.z();

  if(is_attitude_valid)
  {
    publishAttitudeEst(msgAtt);

    {
      std::lock_guard<std::mutex> lock(attitude_msg_mutex_);
      latest_attitude_ = msgAtt;
      is_attitude_valid_ = is_attitude_valid;
    }
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "IMU NOISE CALLBACK called");
}
//}

/* callbackMag()//{ */
void MrsUavFcuApi::callbackMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
{
    /*Extract time from msg*/
    rclcpp::Time sim_time = msg->header.stamp;

    /*Translate magnetometer*/
    umsg_sensors_mag_t msgMag;
    translateMag(msg, msgMag, sim_time);

    /*Swap mag direction*/
    //float mag_x = msgMag.mag[0];
    //msgMag.mag[0] = msgMag.mag[1];
    //msgMag.mag[1] = mag_x;

    /*Update attitude estimator*/
    {
      std::lock_guard<std::mutex> lock(attitude_est_mutex_);
      attitude_estimator_.UpdateMag(msgMag);
    }

    RCLCPP_INFO_ONCE(node_->get_logger(),"[FcuBinder]: MAG CALLBACK called");
}
//}

/* callbackRangefinder() //{ */

void MrsUavFcuApi::callbackRangefinder(const sensor_msgs::msg::Range::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting rangefinder");

  if (_capabilities_.produces_distance_sensor) {

    common_handlers_->publishers.publishDistanceSensor(*msg);
  }
}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void MrsUavFcuApi::timerMain() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "main timer spinning");

  publishBatteryState();

  publishRC();

  timeoutInputs();
}

//}

// | ------------------------- methods ------------------------ |

// translateImu//{
void MrsUavFcuApi::translateImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg, umsg_sensors_imu_t &msgImu, rclcpp::Time &sim_time)
{
    static double index = 0;
    /*Set payload*/
    msgImu.accel[0] = static_cast<float>(msg->linear_acceleration.x / GRAV_CONST);
    msgImu.accel[1] = static_cast<float>(msg->linear_acceleration.y / GRAV_CONST);
    msgImu.accel[2] = static_cast<float>(msg->linear_acceleration.z / GRAV_CONST);
    msgImu.gyro[0] = static_cast<float>(msg->angular_velocity.x);
    msgImu.gyro[1] = static_cast<float>(msg->angular_velocity.y);
    msgImu.gyro[2] = static_cast<float>(msg->angular_velocity.z);
    msgImu.temperature = index;
    msgImu.timestamp = static_cast<uint64_t>(sim_time.nanoseconds()*1e-3); // Convert to microsecond
}
//}

// translateMag//{
void MrsUavFcuApi::translateMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg, umsg_sensors_mag_t &msgMag, rclcpp::Time &sim_time)
{
    /*Set payload*/
    msgMag.mag[0] = static_cast<float>(msg->magnetic_field.x);
    msgMag.mag[1] = static_cast<float>(msg->magnetic_field.y);
    msgMag.mag[2] = static_cast<float>(msg->magnetic_field.z);
    msgMag.timestamp = static_cast<uint64_t>(sim_time.nanoseconds()*1e-3); // Convert to microseconds
}
//}

void MrsUavFcuApi::publishAttitudeEst(const umsg_estimation_attitude_t &msg)
{
  if (!is_initialized_)
  {
      return;
  }
  
  /*----Publish orientation ----*/
      geometry_msgs::msg::QuaternionStamped orientation;
      // Direct 64-bit integer scale to avoid implicit floating-point conversions
      orientation.header.stamp = rclcpp::Time(static_cast<int64_t>(msg.timestamp) * 1000LL, RCL_ROS_TIME);

      // Zero-allocation assignment using the pre-cached frame identifier
      orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_;

      // Mathematical Safety Guard: Enforce normalization to protect downstream estimators
      Eigen::Quaternion<double> q_eig(
          static_cast<double>(msg.w),
          static_cast<double>(msg.x),
          static_cast<double>(msg.y),
          static_cast<double>(msg.z)
      );
      q_eig.normalize(); 

      // Clean assignment to target ROS message fields
      orientation.quaternion.x = q_eig.x();
      orientation.quaternion.y = q_eig.y();
      orientation.quaternion.z = q_eig.z();
      orientation.quaternion.w = q_eig.w();

      if(_publish_orientation_com_filt_)
      {
        ph_orientation_com_filt_.publish(orientation);
      }
      
  /*---- Publish angular velocity ----*/

      geometry_msgs::msg::Vector3Stamped angular_velocity;

      angular_velocity.header.stamp = rclcpp::Time(static_cast<int64_t>(msg.timestamp) * 1000LL, RCL_ROS_TIME);
      angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
      
      geometry_msgs::msg::Vector3 v;
      v.x = static_cast<double>(msg.att_rate[0]);
      v.y = static_cast<double>(msg.att_rate[1]);
      v.z = static_cast<double>(msg.att_rate[2]);
      angular_velocity.vector = v;

      if(_publish_ang_vel_com_filt_)
      {
        ph_ang_vel_com_filt_.publish(angular_velocity);
      }
};

/* publishBatteryState() //{ */

void MrsUavFcuApi::publishBatteryState(void) {

  if (_capabilities_.produces_battery_state) {

    sensor_msgs::msg::BatteryState msg;

    msg.capacity = 100;
    msg.current  = 10.0;
    msg.voltage  = 15.8;
    msg.charge   = 0.8;

    common_handlers_->publishers.publishBatteryState(msg);
  }
}

//}

/* publishRC() //{ */

void MrsUavFcuApi::publishRC(void) {

  if (_capabilities_.produces_rc_channels) {

    mrs_msgs::msg::HwApiRcChannels rc;

    rc.stamp = clock_->now();

    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);

    common_handlers_->publishers.publishRcChannels(rc);
  }
}

//}

/* MrsUavHwApi() //{ */

void MrsUavFcuApi::timeoutInputs(void) {

  auto last_cmd_time = mrs_lib::get_mutexed(mutex_last_cmd_time_, last_cmd_time_);

  if (last_cmd_time_.seconds() > 0 && (clock_->now() - last_cmd_time).seconds() > _input_timeout_) {
    offboard_ = false;
    mode_     = "NORMAL";
  }
}

//}

} // namespace mrs_uav_fcu_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_fcu_api::MrsUavFcuApi, mrs_uav_hw_api::MrsUavHwApi)