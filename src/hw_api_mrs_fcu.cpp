/* includes //{ */

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

#include "serial_api.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <umsg.h>
#include <umsg_classes.h>

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
        std::shared_ptr<SerialApi> ser_;
        rclcpp::Node::SharedPtr node_;
        std::string UTM_zone;

        rclcpp::CallbackGroup::SharedPtr cbgrp_subs_;

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

        void publishImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg, rclcpp::Time &sim_time);
        void publishMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg, rclcpp::Time &sim_time);
        void publishAltitude(const nav_msgs::msg::Odometry::ConstSharedPtr msg, rclcpp::Time &sim_time);
        void publishGps(const nav_msgs::msg::Odometry::ConstSharedPtr msg, rclcpp::Time &sim_time);

        // | ----------------------- publishers ----------------------- |
        mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd> ph_actuator_cmd_;

    public:
        void Init(const rclcpp::Node::SharedPtr parent_node, std::shared_ptr<SerialApi> ser, mrs_lib::ParamLoader& param_loader);
        bool ParseMessage(umsg_MessageToTransfer &msg);
    };

    void hitl_binder::Init(const rclcpp::Node::SharedPtr parent_node, std::shared_ptr<SerialApi> ser, mrs_lib::ParamLoader& param_loader)
    {
        /*Asign Node and Serial class instance*/
        node_ = parent_node;
        ser_ = ser;

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
        sh_imu_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/hitl/imu", &hitl_binder::callbackIMU, this);
        sh_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/hitl/odom", &hitl_binder::callbackOdometry, this);
        //sh_rangefinder_ = mrs_lib::SubscriberHandler<sensor_msgs::Range>(shopts, "/~/hitl/rangefinder", &hitl_binder::callbackRangeFinder, this);
        sh_altitude_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/hitl/altitude", &hitl_binder::callbackAltitude, this);
        sh_mag_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>(shopts, "~/hitl/magnetometer", &hitl_binder::callbackMag, this);

        /*Initialize publishers*/
        ph_actuator_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd>(node_, "~/hitl/actuators_cmd");

        RCLCPP_INFO(node_->get_logger(),"Subscribers and Publishers initialized");
    };

    /*| ------------------------- Publishers ------------------------- |*/

    // PublishImu//{
    void hitl_binder::publishImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg, rclcpp::Time &sim_time)
    { /*//{*/
        static double index = 0;

        /*Set header*/
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_IMU;

        /*Set payload*/
        out.s.sensors.imu.accel[0] = static_cast<float>(msg->linear_acceleration.x / GRAV_CONST);
        out.s.sensors.imu.accel[1] = static_cast<float>(msg->linear_acceleration.y / GRAV_CONST);
        out.s.sensors.imu.accel[2] = static_cast<float>(msg->linear_acceleration.z / GRAV_CONST);

        out.s.sensors.imu.gyro[0] = static_cast<float>(msg->angular_velocity.x);
        out.s.sensors.imu.gyro[1] = static_cast<float>(msg->angular_velocity.y);
        out.s.sensors.imu.gyro[2] = static_cast<float>(msg->angular_velocity.z);
        out.s.sensors.imu.timestamp = ser_->RosToFcu(sim_time);
        out.s.sensors.imu.temperature = index;

        /*Increase index counter*/
        index += 1;

        /*Set message length and CRC*/
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_imu_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);

        /*Send message*/
        ser_->sendPacket(out);
    } /*//}*/ /*//}*/

    void hitl_binder::publishMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg, rclcpp::Time &sim_time)
    {
        /*Set header*/
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_MAG;

        //RCLCPP_INFO(node_->get_logger(), "[FCU BINDER] %f %f %f",R(0,0),R(1,0),R(2,0));

        /*Set payload*/
        out.s.sensors.mag.mag[0] = static_cast<float>(msg->magnetic_field.x);
        out.s.sensors.mag.mag[1] = static_cast<float>(msg->magnetic_field.y);
        out.s.sensors.mag.mag[2] = static_cast<float>(msg->magnetic_field.z);
        out.s.sensors.mag.timestamp = ser_->RosToFcu(sim_time);

        /*Set message length and CRC*/
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_mag_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);

        /*Send message*/
        ser_->sendPacket(out);
    }

    void hitl_binder::publishAltitude(const nav_msgs::msg::Odometry::ConstSharedPtr msg, rclcpp::Time &sim_time)
    {
        /*Set header*/
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_ALTIMETER;

        /*Set payload*/
        out.s.sensors.altimeter.altitude = static_cast<float>(msg->pose.pose.position.z);
        out.s.sensors.altimeter.timestamp = ser_->RosToFcu(sim_time);

        /*Set message length and CRC*/
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_altimeter_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);

        /*Send message*/
        ser_->sendPacket(out);
    }

    void hitl_binder::publishGps(const nav_msgs::msg::Odometry::ConstSharedPtr msg, rclcpp::Time &sim_time)
    {
        /*Set header*/
        umsg_MessageToTransfer out;
        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_SENSORS;
        out.s.msg_type = SENSORS_GPS;

        /*Set payload*/
        out.s.sensors.gps.timestamp = ser_->RosToFcu(sim_time);
        out.s.sensors.gps.fixType = FIX_3D;
        out.s.sensors.gps.hELPS = msg->pose.pose.position.z;
        out.s.sensors.gps.hMSL = msg->pose.pose.position.z;

        out.s.sensors.gps.reserved = 0;
        out.s.sensors.gps.numSV = 20;

        /*Convert local cartesian to global GNSS coordinates*/
        double UTMNorth, UTMEast;
        UTMEast = startX + msg->pose.pose.position.x;
        UTMNorth = startY + msg->pose.pose.position.y;
        double lat, lon;
        mrs_lib::UTMtoLL(UTMNorth, UTMEast, UTM_zone, lat, lon);

        /*Set payload*/
        out.s.sensors.gps.lat = lat;
        out.s.sensors.gps.lon = lon;

        out.s.sensors.gps.CRCValid = 1;
        out.s.sensors.gps.DataValid = 1;
        out.s.sensors.gps.gnssFixOk = 1;

        //Eigen::Vector3d vel_body = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        //Eigen::Quaternion<double> q = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        //Eigen::Matrix3d R = q.toRotationMatrix();
        //Eigen::Vector3d vel_world = R * vel_body;

        // Note: GPS velocity is typically in the body frame, but here we are sending world frame velocity to replicate GPS velocity noise, which is independent of the drone's orientation
        // GPS velocity format is NED (north, east, down), hence the negation of Z component
        out.s.sensors.gps.vel[0] = msg->twist.twist.linear.y;
        out.s.sensors.gps.vel[1] = msg->twist.twist.linear.x;
        out.s.sensors.gps.vel[2] = -msg->twist.twist.linear.z;

        /*Set message length and CRC*/
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_gps_t) + 1;
        out.s.len = len;
        out.raw[len - 1] = umsg_calcCRC(out.raw, len - 1);

        /*Send message*/
        ser_->sendPacket(out);
    }

    /*| ------------------------- callbacks ------------------------- |*/

    void hitl_binder::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }
        /*Fill the packet header*/
        umsg_MessageToTransfer notifyMsg;
        notifyMsg.s.sync0 = 'M';
        notifyMsg.s.sync1 = 'R';
        notifyMsg.s.msg_class = UMSG_SENSORS;
        notifyMsg.s.msg_type = SENSORS_NOTIFYSENSORDATA;

        /*Set Payload*/
        notifyMsg.s.sensors.notifySensorData.imu = 0;
        notifyMsg.s.sensors.notifySensorData.altimeter = 0;
        notifyMsg.s.sensors.notifySensorData.baro = 0;
        notifyMsg.s.sensors.notifySensorData.GPS = 0;
        notifyMsg.s.sensors.notifySensorData.magnetometer = 0;

        /*Get time from msg and publish gps*/
        rclcpp::Time sim_time = msg->header.stamp;
        publishGps(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.GPS = 1;
        RCLCPP_INFO_ONCE(node_->get_logger(), "[HITLBinder]: GPS CALLBACK called");

        /*Set time, length and CRC*/
        notifyMsg.s.sensors.notifySensorData.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);

        /*Send notifyMsg message*/
        ser_->sendPacket(notifyMsg);
    }

    void hitl_binder::callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }

        /*Extract time from msg*/
        rclcpp::Time sim_time = msg->header.stamp;

        /*Fill packet header*/
        umsg_MessageToTransfer notifyMsg;
        notifyMsg.s.sync0 = 'M';
        notifyMsg.s.sync1 = 'R';
        notifyMsg.s.msg_class = UMSG_SENSORS;
        notifyMsg.s.msg_type = SENSORS_NOTIFYSENSORDATA;
        notifyMsg.s.sensors.notifySensorData.imu = 0;
        notifyMsg.s.sensors.notifySensorData.altimeter = 0;
        notifyMsg.s.sensors.notifySensorData.baro = 0;
        notifyMsg.s.sensors.notifySensorData.GPS = 0;
        notifyMsg.s.sensors.notifySensorData.magnetometer = 0;

        /*Publish Imu*/
        publishImu(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.imu = 1;

        RCLCPP_INFO_ONCE(node_->get_logger(),"[HITLBinder]: IMU CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);

        /*Send notifyMsg message*/
        ser_->sendPacket(notifyMsg);
        //  RCLCPP_INFO(node_->get_logger(),"[FcuBinder]: IMU Duration %d",diff_to_now.nanoseconds());
        //  todo send the message over the serial
    }

    void hitl_binder::callbackRangeFinder(const sensor_msgs::msg::Range::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        RCLCPP_WARN_ONCE(node_->get_logger(),"rangefinder callback not yet implemented");
    }

    void hitl_binder::callbackAltitude(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }
        // fill the packet header
        rclcpp::Time sim_time = msg->header.stamp;

        umsg_MessageToTransfer notifyMsg;
        notifyMsg.s.sync0 = 'M';
        notifyMsg.s.sync1 = 'R';
        notifyMsg.s.msg_class = UMSG_SENSORS;
        notifyMsg.s.msg_type = SENSORS_NOTIFYSENSORDATA;
        notifyMsg.s.sensors.notifySensorData.imu = 0;
        notifyMsg.s.sensors.notifySensorData.altimeter = 0;
        notifyMsg.s.sensors.notifySensorData.baro = 0;
        notifyMsg.s.sensors.notifySensorData.GPS = 0;
        notifyMsg.s.sensors.notifySensorData.magnetometer = 0;

        publishAltitude(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.altimeter = 1;
        RCLCPP_INFO_ONCE(node_->get_logger(), "[HITLBinder]: Altitude CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser_->sendPacket(notifyMsg);
        //RCLCPP_INFO(node_->get_logger(), "[FcuBinder]: IMU Duration %d",diff_to_now.nanoseconds());
        //  todo send the message over the serial
    }

    void hitl_binder::callbackMag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }
        // fill the packet header
        rclcpp::Time sim_time = msg->header.stamp;

        umsg_MessageToTransfer notifyMsg;
        notifyMsg.s.sync0 = 'M';
        notifyMsg.s.sync1 = 'R';
        notifyMsg.s.msg_class = UMSG_SENSORS;
        notifyMsg.s.msg_type = SENSORS_NOTIFYSENSORDATA;
        notifyMsg.s.sensors.notifySensorData.imu = 0;
        notifyMsg.s.sensors.notifySensorData.altimeter = 0;
        notifyMsg.s.sensors.notifySensorData.baro = 0;
        notifyMsg.s.sensors.notifySensorData.GPS = 0;
        notifyMsg.s.sensors.notifySensorData.magnetometer = 0;

        publishMag(msg, sim_time);
        notifyMsg.s.sensors.notifySensorData.magnetometer = 1;
        RCLCPP_INFO_ONCE(node_->get_logger(),"[FcuBinder]: mag CALLBACK called");

        notifyMsg.s.sensors.notifySensorData.timestamp = ser_->RosToFcu(sim_time);
        uint32_t len = UMSG_HEADER_SIZE;
        len += sizeof(umsg_sensors_notifySensorData_t) + 1;
        notifyMsg.s.len = len;
        notifyMsg.raw[len - 1] = umsg_calcCRC(notifyMsg.raw, len - 1);
        ser_->sendPacket(notifyMsg);
        // RCLCPP_INFO(node_->get_logger(),"[FcuBinder]: IMU Duration %d",diff_to_now.nanoseconds());
        //  todo send the message over the serial
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
                        mrs_msgs::msg::HwApiActuatorCmd cmd;
                        umsg_control_DshotMessage_t DshotMessage = msg.s.control.DshotMessage;
                        cmd.stamp = ser_->FcuToRos(DshotMessage.timestamp);

                        for (size_t i = 0; i < 4; i++)
                        {
                            cmd.motors.push_back(static_cast<float>(DshotMessage.channels[i] - 48) / 2048.);
                        }
                        ph_actuator_cmd_.publish(cmd);
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

    class MrsUavFcuApi : public mrs_uav_hw_api::MrsUavHwApi
    {

    public:
        MrsUavFcuApi() {}
        ~MrsUavFcuApi() override{}

        void initialize(const rclcpp::Node::SharedPtr &parent_node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

        void destroy() override;

        rclcpp::CallbackGroup::SharedPtr cbgrp_subs_;

        // | --------------------- status methods --------------------- |

        mrs_msgs::msg::HwApiStatus getStatus();
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
        // | -------------------- service callbacks ------------------- |

        std::tuple<bool, std::string> callbackArming(const bool &request);
        std::tuple<bool, std::string> callbackOffboard(void);

    private:
        bool is_initialized_ = false;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Clock::SharedPtr clock_;

        std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

        // | ---------------------- publishers ----------------------- |
        // outside of common handlers because of HITL
        std::shared_ptr<mrs_lib::PublisherHandler<geometry_msgs::msg::PointStamped>>   ph_position_fcu_;

        // | ---------------------- subscribers ----------------------- |

        //mrs_lib::SubscriberHandler<> sh_ground_truth_;
        //mrs_lib::SubscriberHandler<> sh_rtk_;

        // | ----------------------- parameters ----------------------- |

        mrs_msgs::msg::HwApiCapabilities _capabilities_;

        std::string _uav_name_;
        std::string _body_frame_name_;
        std::string _world_frame_name_;

        double _comms_timeout_;

        bool _simulation_;

        std::shared_ptr<SerialApi> ser_;
        hitl_binder hitl_binder_;

        // output methods for rtk
        void publishGroundTruth(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

        void publishRTK(const std::shared_ptr<mrs_modules_msgs::msg::Bestpos> msg);

        double RCChannelToRange(const double &rc_value);

        // normal output methods

        void publishState(const umsg_state_UAV_state_t &msg);
        void publishAttitude(const umsg_estimation_attitude_t &msg);
        void publishOdometryLocal(const umsg_estimation_position_t &msg);
        void publishNavsatFix(umsg_sensors_gps_t &msg);
        void publishDistanceSensor(const sensor_msgs::msg::Range::ConstSharedPtr msg); // not yet implemented
        void publishImu(const umsg_sensors_imu_t &msg);
        void publishMagnetometer(const umsg_sensors_mag_t &msg);
        void publishMagneticField(const umsg_sensors_mag_t &msg);
        void publishRC(const umsg_control_sBusPacket_t &msg);
        void publishAltitude(const umsg_sensors_altimeter_t &msg);
        void publishGpsStatusRaw(const umsg_sensors_gps_t &msg);
        void publishBattery(); // not yet implemented

        std::thread parser_thread_;
        void messageParser();

        bool ParseMessage(umsg_MessageToTransfer &msg);
        // | ------------------------ variables ----------------------- |
        // | ------------------------ variables ----------------------- |

        std::atomic<bool> offboard_ = false;
        std::string mode_;
        std::atomic<bool> armed_ = false;
        std::atomic<bool> connected_ = false;
        std::mutex mutex_status_;

        std::mutex odometry_mutex_;
        Eigen::Matrix3d R_orientation_;
        std::atomic<bool> orientation_initialized_ = false;
        std::string uav_name_;
        nav_msgs::msg::Odometry odometry_drone_est_;
    };

    //}

    /* initialize() //{ */

    void MrsUavFcuApi::initialize(const rclcpp::Node::SharedPtr &parent_node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers)
    {
        node_  = parent_node;
        clock_ = node_->get_clock();

        cbgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        common_handlers_ = common_handlers;

        _uav_name_ = common_handlers->getUavName();
        _body_frame_name_ = common_handlers->getBodyFrameName();
        _world_frame_name_ = common_handlers->getWorldFrameName();

        _capabilities_.api_name = "FcuApi";

        // | ------------------- loading parameters ------------------- |

        mrs_lib::ParamLoader local_param_loader(node_, "MrsUavHwApi");

        std::string custom_config_path;

        common_handlers_->main_param_loader->loadParam("custom_config", custom_config_path);

        if (custom_config_path != "")
        {
            local_param_loader.addYamlFile(custom_config_path);    
        }

        std::vector<std::string> config_files;
        common_handlers_->main_param_loader->loadParamReusable("configs", config_files);

        if (!common_handlers_->main_param_loader->loadedSuccessfully())
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
            rclcpp::shutdown();
            exit(1);
        }

        for (auto config_file : config_files)
        {
            RCLCPP_INFO(node_->get_logger(), "loading config files '%s'", config_file.c_str());
            local_param_loader.addYamlFile(config_file);
        }

        // ask what this does
        local_param_loader.loadParam("comms_timeout", _comms_timeout_);

        // ask what this does
        local_param_loader.loadParam("simulation", _simulation_);

        local_param_loader.loadParam("inputs/control_group", (bool &)_capabilities_.accepts_control_group_cmd);
        local_param_loader.loadParam("inputs/attitude_rate", (bool &)_capabilities_.accepts_attitude_rate_cmd);
        local_param_loader.loadParam("inputs/attitude", (bool &)_capabilities_.accepts_attitude_cmd);

        local_param_loader.loadParam("outputs/distance_sensor", (bool &)_capabilities_.produces_distance_sensor);
        local_param_loader.loadParam("outputs/gnss", (bool &)_capabilities_.produces_gnss);
        local_param_loader.loadParam("outputs/gnss_status", (bool &)_capabilities_.produces_gnss_status);
        local_param_loader.loadParam("outputs/rtk", (bool &)_capabilities_.produces_rtk);
        local_param_loader.loadParam("outputs/ground_truth", (bool &)_capabilities_.produces_ground_truth);
        local_param_loader.loadParam("outputs/imu", (bool &)_capabilities_.produces_imu);
        local_param_loader.loadParam("outputs/altitude", (bool &)_capabilities_.produces_altitude);
        local_param_loader.loadParam("outputs/magnetometer_heading", (bool &)_capabilities_.produces_magnetometer_heading);
        local_param_loader.loadParam("outputs/magnetic_field", (bool &)_capabilities_.produces_magnetic_field);
        local_param_loader.loadParam("outputs/rc_channels", (bool &)_capabilities_.produces_rc_channels);
        local_param_loader.loadParam("outputs/battery_state", (bool &)_capabilities_.produces_battery_state);
        local_param_loader.loadParam("outputs/position", (bool &)_capabilities_.produces_position);
        local_param_loader.loadParam("outputs/orientation", (bool &)_capabilities_.produces_orientation);
        local_param_loader.loadParam("outputs/velocity", (bool &)_capabilities_.produces_velocity);
        local_param_loader.loadParam("outputs/angular_velocity", (bool &)_capabilities_.produces_angular_velocity);
        local_param_loader.loadParam("outputs/odometry", (bool &)_capabilities_.produces_odometry);

        if (!local_param_loader.loadedSuccessfully())
        {
            RCLCPP_ERROR(node_->get_logger(), "[MrsUavFcuApi]: Could not load all parameters!");
            rclcpp::shutdown();
        }

        // | ----------------------- subscribers ---------------------- |

        mrs_lib::SubscriberHandlerOptions shopts;

        shopts.node                                = node_;
        shopts.node_name                           = "MrsHwFcuApi";
        shopts.no_message_timeout                  = mrs_lib::no_timeout;
        shopts.threadsafe                          = true;
        shopts.autostart                           = true;
        shopts.subscription_options.callback_group = cbgrp_subs_;

        if (_simulation_)
        {
            // sh_ground_truth_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "ground_truth_in", &MrsUavFcuApi::callbackGroundTruth, this);
        }
        else /*!_simulation_*/
        {
            // sh_rtk_ = mrs_lib::SubscribeHandler<mrs_modules_msgs::msg::Bestpos>(shopts, "rtk_in", &MrsUavFcuApi::callbackRTK, this);
        }

        // | ----------------------- publishers ----------------------- |

        //Added for time DEBUGGING
        ph_position_fcu_ = std::make_shared<mrs_lib::PublisherHandler<geometry_msgs::msg::PointStamped>>(node_, "~/HITL/FCU_position_raw_time");

        // | ----------------------- finish init ---------------------- |

        /*Init Serial api communication*/
        std::string serial_port;
        std::string serial_port_back_up;
        int baud_rate;
        local_param_loader.loadParam("serial_port", serial_port);
        local_param_loader.loadParam("serial_port_back_up", serial_port_back_up);
        local_param_loader.loadParam("baud_rate", baud_rate);

        ser_ = std::make_shared<SerialApi>(node_, serial_port, serial_port_back_up, baud_rate);
        ser_->startSerialApiThreads();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ser_->startSyncTimer();

        /*Init HITL binder*/
        if (_simulation_)
        {
            hitl_binder_.Init(node_, ser_, local_param_loader);
        }

        RCLCPP_INFO(node_->get_logger(),"[MrsUavFcuApi]: initialized");

        parser_thread_ = std::thread([this]
                                     { this->messageParser(); });
        is_initialized_ = true;
    }

    //}

    /*destroy() //{*/
    void MrsUavFcuApi::destroy() {}
    //}

    /* getStatus() //{ */

    mrs_msgs::msg::HwApiStatus MrsUavFcuApi::getStatus()
    {

        mrs_msgs::msg::HwApiStatus status;

        status.stamp = clock_->now();
        status.armed = armed_;
        status.offboard = offboard_;
        status.connected = connected_;
        status.mode = mode_;

        return status;
    }

    //}

    /* getCapabilities() //{ */

    mrs_msgs::msg::HwApiCapabilities MrsUavFcuApi::getCapabilities()
    {
        _capabilities_.stamp = clock_->now();

        return _capabilities_;
    }

    //}

    /* callbackAttitudeRateCmd() //{ */

    bool MrsUavFcuApi::callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg)
    {

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting attitude rate cmd");

        if (!_capabilities_.accepts_attitude_rate_cmd)
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1.0, "[MrsUavFcuApi]: attitude rate input is not enabled in the config file");
            return false;
        }

        umsg_MessageToTransfer out;

        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_OFFBOARD;
        out.s.msg_type = OFFBOARD_RATECMD;
        out.s.offboard.RateCmd.roll_rate = msg->body_rate.x;
        out.s.offboard.RateCmd.pitch_rate = msg->body_rate.y;
        out.s.offboard.RateCmd.yaw_rate = msg->body_rate.z;
        out.s.offboard.RateCmd.throttle = msg->throttle;
        out.s.offboard.RateCmd.timestamp = ser_->RosToFcu(msg->stamp);
        out.s.len = UMSG_HEADER_SIZE + sizeof(umsg_offboard_RateCmd_t) + 1;
        out.raw[out.s.len - 1] = umsg_calcCRC(out.raw, out.s.len - 1);
        ser_->sendPacket(out);

        return true;
    }

    //}

    /* callbackAttitudeCmd() //{ */

    bool MrsUavFcuApi::callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg)
    {

        RCLCPP_INFO_ONCE(node_->get_logger(), "[MrsUavFcuApi]: getting attitude cmd");

        if (!_capabilities_.accepts_attitude_cmd)
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1.0, "[MrsUavFcuApi]: attitude input is not enabled in the config file");
            return false;
        }
        umsg_MessageToTransfer out;

        out.s.sync0 = 'M';
        out.s.sync1 = 'R';
        out.s.msg_class = UMSG_OFFBOARD;
        out.s.msg_type = OFFBOARD_ATTITUDECMD;
        out.s.offboard.AttitudeCmd.w = msg->orientation.w;
        out.s.offboard.AttitudeCmd.x = msg->orientation.x;
        out.s.offboard.AttitudeCmd.y = msg->orientation.y;
        out.s.offboard.AttitudeCmd.z = msg->orientation.z;
        out.s.offboard.AttitudeCmd.throttle = msg->throttle;
        out.s.offboard.AttitudeCmd.timestamp = ser_->RosToFcu(msg->stamp);
        out.s.len = UMSG_HEADER_SIZE + sizeof(umsg_offboard_AttitudeCmd_t) + 1;
        out.raw[out.s.len - 1] = umsg_calcCRC(out.raw, out.s.len - 1);
        ser_->sendPacket(out);

        return true;
    }

    //}

    /* callbackArming() //{ */

    std::tuple<bool, std::string> MrsUavFcuApi::callbackArming([[maybe_unused]] const bool &request)
    {

        std::stringstream ss;

        // when REALWORLD AND ARM:=TRUE
        if (!_simulation_ && request)
        {

            ss << "can not arm by service when not in simulation! You should arm the drone by the RC controller only!";
            RCLCPP_ERROR_STREAM_THROTTLE( node_->get_logger(), *clock_, 1.0, "[Px4Api]: " << ss.str());

            return {false, "ss.str()"};
        }

        umsg_MessageToTransfer msg;
        msg.s.sync0 = 'M';
        msg.s.sync1 = 'R';
        msg.s.msg_class = UMSG_STATE;
        msg.s.msg_type = STATE_STATECHANGEREQUEST;
        msg.s.state.stateChangeRequest.requestedState = request ? UAV_FLYING : UAV_DISARMED;
        msg.s.state.stateChangeRequest.timestamp = ser_->RosToFcu(clock_->now());
        msg.s.len = UMSG_HEADER_SIZE + sizeof(umsg_state_stateChangeRequest_t) + 1;
        msg.raw[msg.s.len - 1] = umsg_calcCRC(msg.raw, msg.s.len - 1);
        ser_->sendPacket(msg);

        // TODO maybe there is a confirmation mechanism needed?
        RCLCPP_INFO(node_->get_logger(),"[FcuApi]: calling for %s", request ? "arming" : "disarming");

        return {true, "ss.str()"};
    }

    //}

    /* callbackOffboard() //{ */

    std::tuple<bool, std::string> MrsUavFcuApi::callbackOffboard(void)
    {
        umsg_MessageToTransfer msg;
        msg.s.sync0 = 'M';
        msg.s.sync1 = 'R';
        msg.s.msg_class = UMSG_STATE;
        msg.s.msg_type = STATE_MODECHANGEREQUEST;
        msg.s.state.modeChangeRequest.requestedMode = OFFBOARD;
        msg.s.state.modeChangeRequest.timestamp = ser_->RosToFcu(clock_->now());
        msg.s.len = UMSG_HEADER_SIZE + sizeof(umsg_state_modeChangeRequest_t) + 1;
        msg.raw[msg.s.len - 1] = umsg_calcCRC(msg.raw, msg.s.len - 1);
        ser_->sendPacket(msg);

        // TODO maybe there is a confirmation mechanism needed?
        RCLCPP_INFO(node_->get_logger(),"[FcuApi]: calling for offboard mode");

        return {true, "ss.str()"};
    }

    //}

    // | ------------------- additional methods ------------------- |

    /* timeoutMavrosState() //{ */
    /*
    void MrsUavFcuApi::timeoutMavrosState([[maybe_unused]] const std::string &topic, const ros::Time &last_msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        if (!sh_mavros_state_.hasMsg())
        {
            return;
        }

        ros::Duration time = ros::Time::now() - last_msg;

        if (time.toSec() > _comms_timeout_)
        {

            {
                std::scoped_lock lock(mutex_status_);

                connected_ = false;
                offboard_ = false;
                armed_ = false;
                mode_ = "";
            }

            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: Have not received Mavros state for more than '%.3f s'", time.toSec());
        }
        else
        {

            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: Not recieving Mavros state message for '%.3f s'! Setup the PixHawk SD card!!", time.toSec());
            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: This could be also caused by the not being PixHawk booted properly due to, e.g., antispark connector jerkyness.");
            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: The Mavros state should be supplied at 100 Hz to provided fast refresh rate on the state of the OFFBOARD mode.");
            ROS_WARN_THROTTLE(1.0, "[MrsUavFcuApi]: If missing, the UAV could be disarmed by safety routines while not knowing it has switched to the MANUAL mode.");
        }
    }
    */
    //}

    /* RCChannelToRange() //{ */

    double MrsUavFcuApi::RCChannelToRange(const double &rc_value)
    {

        double tmp_0_to_1 = (rc_value - double(PWM_MIN)) / (double(PWM_RANGE));

        if (tmp_0_to_1 > 1.0)
        {
            tmp_0_to_1 = 1.0;
        }
        else if (tmp_0_to_1 < 0.0)
        {
            tmp_0_to_1 = 0.0;
        }

        return tmp_0_to_1;
    }

    //}

    // | ------------------------ callbacks ----------------------- |

    /* //{ callbackMavrosState() */

    void MrsUavFcuApi::publishState(const umsg_state_UAV_state_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting Mavros state");

        {
            std::scoped_lock lock(mutex_status_);

            offboard_ = msg.control_mode == OFFBOARD;
            armed_ = msg.state == UAV_FLYING;
            connected_ = true;
            switch (msg.control_mode)
            {
            case MANUAL:
                mode_ = "MANUAL";
                break;
            case AUTONOMOUS:
                mode_ = "AUTONOMOUS";
                break;
            case OFFBOARD:
                mode_ = "OFFBOARD";
                break;
            }
        }
        // | ----------------- publish the diagnostics ---------------- |

        mrs_msgs::msg::HwApiStatus status;

        {
            std::scoped_lock lock(mutex_status_);

            status.stamp = ser_->FcuToRos(msg.timestamp);
            status.armed = armed_;
            status.offboard = offboard_;
            status.connected = connected_;
            status.mode = mode_;
        }

        common_handlers_->publishers.publishStatus(status);
    }

    //}

    /* callbackOdometryLocal() //{ */

    void MrsUavFcuApi::publishOdometryLocal(const umsg_estimation_position_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(), "[MrsUavFcuApi]: getting Mavros's local odometry");

        // | -------------------- publish position -------------------- |

        if (_capabilities_.produces_position)
        {

            geometry_msgs::msg::PointStamped position;

            position.header.stamp = ser_->FcuToRos(msg.timestamp);
            position.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
            geometry_msgs::msg::Point p;
            p.x = msg.position[0];
            p.y = msg.position[1];
            p.z = msg.position[2];
            position.point = p;

            common_handlers_->publishers.publishPosition(position);
        }

        // | ------------------- publish orientation ------------------ |

        // | -------------------- publish velocity -------------------- |

        if(orientation_initialized_)
        {
            nav_msgs::msg::Odometry odom;
            Eigen::Matrix3d R_orientation;
            
            { // Optain orination matrix along with angluar velocity in body frame to ensure orientation consistency between angular and linear velocity
                
                std::scoped_lock lock(odometry_mutex_);
                R_orientation = R_orientation_;

                odom.twist.twist.angular.x = odometry_drone_est_.twist.twist.angular.x;
                odom.twist.twist.angular.y = odometry_drone_est_.twist.twist.angular.y;
                odom.twist.twist.angular.z = odometry_drone_est_.twist.twist.angular.z;
            }
            Eigen::Vector3d vel_world(msg.velocity);
            Eigen::Vector3d vel_body = R_orientation.transpose() * vel_world;

            if (_capabilities_.produces_velocity)
            {
                geometry_msgs::msg::Vector3Stamped velocity;

                velocity.header.stamp = ser_->FcuToRos(msg.timestamp);
                //  TODO FIX : you have to
                velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;

                velocity.vector.x = vel_body.x();
                velocity.vector.y = vel_body.y();
                velocity.vector.z = vel_body.z();
                
                common_handlers_->publishers.publishVelocity(velocity);
            }

            // | -------------------- publish odometry -------------------- |

            if (_capabilities_.produces_odometry)
            {
                RCLCPP_ERROR_ONCE(node_->get_logger(),"[ODOMETRY] not yet implemented");
                nav_msgs::msg::Odometry odom;

                odom.header.stamp    = ser_->FcuToRos(msg.timestamp);
                odom.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
                //odom.child_frame_id  = _frame_fcu_;

                /*Get orientatiom matrix (Drone -> world)*/
                odom.pose.pose.orientation = mrs_lib::AttitudeConverter(R_orientation);
                /*Get linear velocity in body frame*/
                odom.twist.twist.linear.x = vel_body.x();
                odom.twist.twist.linear.y = vel_body.y();
                odom.twist.twist.linear.z = vel_body.z();
                
                /*Get position in world frame*/
                odom.pose.pose.position.x = msg.position[0];// - startX;
                odom.pose.pose.position.y = msg.position[1];// - startY;
                odom.pose.pose.position.z = msg.position[2];

                common_handlers_->publishers.publishOdometry(odom);
            }
        }
        //Added publisher for FCU time DEBUGING
        //{    
        geometry_msgs::msg::PointStamped position;
        //Keep the time unchanged
        position.header.stamp = rclcpp::Time(static_cast<int64_t>(msg.timestamp)*1e6); // from ms to ns
        position.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
        geometry_msgs::msg::Point p;
        p.x = msg.position[0];
        p.y = msg.position[1];
        p.z = msg.position[2];
        position.point = p;
        
        ph_position_fcu_->publish(position);
        //}
    }

    //}

    /* callbackNavsatFix() //{ */
    void MrsUavFcuApi::publishNavsatFix(umsg_sensors_gps_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting NavSat fix");

        if (_capabilities_.produces_gnss)
        {
            sensor_msgs::msg::NavSatFix fixmsg;
            fixmsg.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
            fixmsg.header.stamp = ser_->FcuToRos(msg.timestamp);
            fixmsg.altitude = msg.hMSL;
            fixmsg.latitude = msg.lat;
            fixmsg.longitude = msg.lon;
            fixmsg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            fixmsg.status.status = (msg.fixType == FIX_3D) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            common_handlers_->publishers.publishGNSS(fixmsg);
        }
    }

    //}

    /* callbackDistanceSensor() //{ */

    void MrsUavFcuApi::publishDistanceSensor(const sensor_msgs::msg::Range::ConstSharedPtr msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(), "[MrsUavFcuApi]: getting distnace sensor");

        if (_capabilities_.produces_distance_sensor)
        {
            common_handlers_->publishers.publishDistanceSensor(*msg);
        }
    }

    //}

    /* callbackImu() //{ */

    void MrsUavFcuApi::publishImu(const umsg_sensors_imu_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting IMU");

        if (_capabilities_.produces_imu)
        {
            sensor_msgs::msg::Imu imu;
            imu.header.frame_id = _uav_name_ + "/" + _body_frame_name_;

            imu.header.stamp = ser_->FcuToRos(msg.timestamp);
            imu.angular_velocity.x = msg.gyro[0];
            imu.angular_velocity.y = msg.gyro[1];
            imu.angular_velocity.z = msg.gyro[2];
            imu.linear_acceleration.x = msg.accel[0];
            imu.linear_acceleration.y = msg.accel[1];
            imu.linear_acceleration.z = msg.accel[2];
            common_handlers_->publishers.publishIMU(imu);
        }
    }

    //}

    /* callbackCompass() //{ */

    void MrsUavFcuApi::publishMagnetometer(const umsg_sensors_mag_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting magnetometer heading");

        if (_capabilities_.produces_magnetometer_heading)
        {
            mrs_msgs::msg::Float64Stamped mag_out;
            mag_out.header.stamp = ser_->FcuToRos(msg.timestamp);
            mag_out.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
            mag_out.value = 180 / M_PI * std::atan2(msg.mag[1], msg.mag[0]);

            common_handlers_->publishers.publishMagnetometerHeading(mag_out);
        }
    }

    //}

    /* callbackMagneticField() //{ */

    void MrsUavFcuApi::publishMagneticField(const umsg_sensors_mag_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting magnetic field");

        if (_capabilities_.produces_magnetic_field)
        {
            sensor_msgs::msg::MagneticField mag;
            mag.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
            mag.header.stamp = ser_->FcuToRos(msg.timestamp);
            mag.magnetic_field.x = static_cast<double>(msg.mag[0]);
            mag.magnetic_field.y = static_cast<double>(msg.mag[1]);
            mag.magnetic_field.z = static_cast<double>(msg.mag[2]);
            common_handlers_->publishers.publishMagneticField(mag);
        }
    }

    //}

    /* callbackRC() //{ */

    void MrsUavFcuApi::publishRC(const umsg_control_sBusPacket_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(), "[MrsUavFcuApi]: getting RC");

        if (_capabilities_.produces_rc_channels)
        {

            mrs_msgs::msg::HwApiRcChannels rc_out;

            rc_out.stamp = ser_->FcuToRos(msg.timestamp);

            for (size_t i = 1; i < 17; i++)
            {
                rc_out.channels.push_back(RCChannelToRange(static_cast<double>(msg.channels[i])));
            }

            common_handlers_->publishers.publishRcChannels(rc_out);
        }
    }

    //}

    /* callbackAltitude() //{ */

    void MrsUavFcuApi::publishAltitude(const umsg_sensors_altimeter_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting Altitude");

        if (_capabilities_.produces_altitude)
        {
            mrs_msgs::msg::HwApiAltitude altitude_out;
            altitude_out.stamp = ser_->FcuToRos(msg.timestamp);
            altitude_out.amsl = static_cast<double>(msg.altitude);

            common_handlers_->publishers.publishAltitude(altitude_out);
        }
    }

    //}

    /* callbackAltitude() //{ */

    void MrsUavFcuApi::publishGpsStatusRaw(const umsg_sensors_gps_t &msg)
    {

        if (!is_initialized_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting Gps Status Raw");

        if (_capabilities_.produces_gnss_status)
        {

            mrs_msgs::msg::GpsInfo gps_info_out;

            gps_info_out.stamp = ser_->FcuToRos(msg.timestamp); // [GPS_FIX_TYPE] GPS fix type
            gps_info_out.fix_type = msg.fixType;                // [GPS_FIX_TYPE] GPS fix type

            gps_info_out.lat = msg.lat; // [deg] Latitude (WGS84, EGM96 ellipsoid)
            gps_info_out.lon = msg.lon;
            gps_info_out.alt = msg.hMSL;   // [m]  (MSL). Positive for up. Not WGS84 altitude.
            gps_info_out.eph = UINT16_MAX; // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
            gps_info_out.epv = UINT16_MAX; // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
            // TODO ASK ABOUT THIS
            gps_info_out.vel = 0; // [m/s] GPS ground speed. If unknown, set to: UINT16_MAX
            gps_info_out.cog = 0;
            // [deg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees.
            gps_info_out.satellites_visible = msg.numSV; // Number of satellites visible. If unknown, set to 255

            gps_info_out.alt_ellipsoid = msg.hELPS; // [m] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
            // TODO ASK ABOUT THIS
            gps_info_out.h_acc = 0;         // [m] Position uncertainty. Positive for up.
            gps_info_out.v_acc = 0;         // [m] Altitude uncertainty. Positive for up.
            gps_info_out.vel_acc = 0;       // [m/s] Speed uncertainty. Positive for up.
            gps_info_out.hdg_acc = 0;       // [deg] Heading / track uncertainty
            gps_info_out.yaw = 0;           // [deg] Yaw in earth frame from north.
            gps_info_out.dgps_num_sats = 0; // Number of DGPS satellites
            gps_info_out.dgps_age = 0;      // [s] Age of DGPS info
            gps_info_out.baseline_dist = 0; // [m] distance to the basestation, not supported by the GPSRAW message

            common_handlers_->publishers.publishGNSSStatus(gps_info_out);
        }
    }

    //}

    void MrsUavFcuApi::publishAttitude(const umsg_estimation_attitude_t &msg)
    {
        if (!is_initialized_)
        {
            return;
        }

        /*----Calculate Rotation matrix and save angular velocity----*/
        Eigen::Quaternion<double> q = Eigen::Quaternion<float>(msg.w, msg.x, msg.y, msg.z).cast<double>();
        q.normalize();
        Eigen::Matrix3d R = q.toRotationMatrix();
        {
            std::scoped_lock lock(odometry_mutex_);
            R_orientation_ = R;

            odometry_drone_est_.twist.twist.angular.x = static_cast<double>(msg.att_rate[0]);
            odometry_drone_est_.twist.twist.angular.y = static_cast<double>(msg.att_rate[1]);
            odometry_drone_est_.twist.twist.angular.z = static_cast<double>(msg.att_rate[2]);
        }
        orientation_initialized_ = true;

        /*----Publish orientation ----*/
        if (_capabilities_.produces_orientation)
        {

            geometry_msgs::msg::QuaternionStamped orientation;

            orientation.header.stamp = ser_->FcuToRos(msg.timestamp);
            orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_;

            Eigen::Quaternion<float> q_eig = Eigen::Quaternion<float>(msg.w, msg.x, msg.y, msg.z).inverse();
            geometry_msgs::msg::Quaternion q;
            q.x = static_cast<double>(q_eig.x());
            q.y = static_cast<double>(q_eig.y());
            q.z = static_cast<double>(q_eig.z());
            q.w = static_cast<double>(q_eig.w());
            orientation.quaternion = q;

            common_handlers_->publishers.publishOrientation(orientation);
        }

        /*---- Publish angular velocity ----*/
        if (_capabilities_.produces_angular_velocity)
        {

            geometry_msgs::msg::Vector3Stamped angular_velocity;

            angular_velocity.header.stamp = ser_->FcuToRos(msg.timestamp);
            angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
            geometry_msgs::msg::Vector3 v;
            v.x = static_cast<double>(msg.att_rate[0]);
            v.y = static_cast<double>(msg.att_rate[1]);
            v.z = static_cast<double>(msg.att_rate[2]);
            angular_velocity.vector = v;

            common_handlers_->publishers.publishAngularVelocity(angular_velocity);
        }
    };

    /* callbackBattery() //{ */

    void MrsUavFcuApi::publishBattery()
    {

        if (!ser_->isSynced())
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting battery");

        if (_capabilities_.produces_battery_state)
        {

            RCLCPP_ERROR(node_->get_logger(),"[pub battery] NOT IMPLEMENTED");
            // common_handlers_->publishers.publishBatteryState(*msg);
        }
    }

    //}

    /* callbackGroundTruth() //{ */

    void MrsUavFcuApi::publishGroundTruth(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting ground truth");

        auto odom = msg;

        // | ------------------ publish ground truth ------------------ |

        if (_capabilities_.produces_ground_truth)
        {

            nav_msgs::msg::Odometry gt = *msg;

            // if frame_id is "/world", "world", "/map" or "map" gazebo reports velocitites in global world frame so we need to transform them to body frame
            if (msg->header.frame_id == "/world" || msg->header.frame_id == "world" || msg->header.frame_id == "/map" || msg->header.frame_id == "map")
            {

                RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: transforming Gazebo ground truth velocities from world to body frame");

                Eigen::Matrix3d R = mrs_lib::AttitudeConverter(msg->pose.pose.orientation);

                Eigen::Vector3d lin_vel_world(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
                Eigen::Vector3d lin_vel_body = R.inverse() * lin_vel_world;

                Eigen::Vector3d angular_vel_world(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
                Eigen::Vector3d angular_vel_body = R.inverse() * angular_vel_world;

                gt.twist.twist.linear.x = lin_vel_body[0];
                gt.twist.twist.linear.y = lin_vel_body[1];
                gt.twist.twist.linear.z = lin_vel_body[2];

                gt.twist.twist.angular.x = angular_vel_body[0];
                gt.twist.twist.angular.y = angular_vel_body[1];
                gt.twist.twist.angular.z = angular_vel_body[2];
            }

            common_handlers_->publishers.publishGroundTruth(gt);
        }

        if (_capabilities_.produces_rtk)
        {
            /*
            double lat;
            double lon;

            mrs_lib::UTMtoLL(msg->pose.pose.position.y + _sim_rtk_utm_y_, msg->pose.pose.position.x + _sim_rtk_utm_x_, _sim_rtk_utm_zone_, lat, lon);

            sensor_msgs::NavSatFix gnss;

            gnss.header.stamp = msg->header.stamp;

            gnss.latitude = lat;
            gnss.longitude = lon;
            gnss.altitude = msg->pose.pose.position.z + _sim_rtk_amsl_;

            mrs_msgs::RtkGps rtk;

            rtk.header.stamp = msg->header.stamp;
            rtk.header.frame_id = "gps";

            rtk.gps.latitude = lat;
            rtk.gps.longitude = lon;
            rtk.gps.altitude = msg->pose.pose.position.z + _sim_rtk_amsl_;
            rtk.gps.covariance[0] = std::pow(0.1, 2);
            rtk.gps.covariance[4] = std::pow(0.1, 2);
            rtk.gps.covariance[8] = std::pow(0.1, 2);

            rtk.fix_type.fix_type = rtk.fix_type.RTK_FIX;

            rtk.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;

            common_handlers_->publishers.publishRTK(rtk);
        */
        }
    }

    //}

    /* callbackRTK() //{ */

    void MrsUavFcuApi::publishRTK(const std::shared_ptr<mrs_modules_msgs::msg::Bestpos> msg)
    {
        if (!ser_->isSynced())
        {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsUavFcuApi]: getting rtk");

        mrs_msgs::msg::RtkGps rtk_msg_out;

        rtk_msg_out.gps.latitude = msg->latitude;
        rtk_msg_out.gps.longitude = msg->longitude;
        rtk_msg_out.gps.altitude = msg->height;

        rtk_msg_out.header.stamp = clock_->now();
        rtk_msg_out.header.frame_id = _uav_name_ + "/" + _body_frame_name_;

        if (msg->position_type == "L1_INT")
        {
            rtk_msg_out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FIX;
        }
        else if (msg->position_type == "L1_FLOAT")
        {
            rtk_msg_out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FLOAT;
        }
        else if (msg->position_type == "PSRDIFF")
        {
            rtk_msg_out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.DGPS;
        }
        else if (msg->position_type == "SINGLE")
        {
            rtk_msg_out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.SPS;
        }
        else if (msg->position_type == "NONE")
        {
            rtk_msg_out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.NO_FIX;
        }

        common_handlers_->publishers.publishRTK(rtk_msg_out);
    }

    bool MrsUavFcuApi::ParseMessage(umsg_MessageToTransfer &msg)
    {
        uint8_t msg_class = msg.s.msg_class;
        uint8_t msg_type = msg.s.msg_type;

        bool parsed = true;

        switch (msg_class)
        {
        case UMSG_SENSORS:
        {
            switch (msg_type)
            {
            case SENSORS_IMU:
                publishImu(msg.s.sensors.imu);
                break;
            case SENSORS_GPS:
            {

                umsg_sensors_gps_t gps = msg.s.sensors.gps;
                publishNavsatFix(gps);
                publishGpsStatusRaw(gps);
            }
            break;
            case SENSORS_ALTIMETER:
                publishAltitude(msg.s.sensors.altimeter);
                break;
            case SENSORS_MAG:
                publishMagneticField(msg.s.sensors.mag);
                publishMagnetometer(msg.s.sensors.mag);
                break;
            default:
                parsed = false;
                break;
            }
        }
        break;
        case UMSG_STATE:
        {
            switch (msg_type)
            {
            case STATE_UAV_STATE:
                publishState(msg.s.state.UAV_state);
                break;

            default:
                parsed = false;
                break;
            }
        }
        break;
        case UMSG_CONTROL:
        {
            switch (msg_type)
            {
            case CONTROL_SBUSPACKET:
                publishRC(msg.s.control.sBusPacket);
                break;

            default:
                parsed = false;
                break;
            }
        }
        break;

        case UMSG_ESTIMATION:
        {
            switch (msg_type)
            {
            case ESTIMATION_ATTITUDE:
            {
                publishAttitude(msg.s.estimation.attitude);
            }
            break;
            case ESTIMATION_POSITION:
                publishOdometryLocal(msg.s.estimation.position);
                break;

            default:
                parsed = false;
                break;
            }
        }
        break;

        default:
            break;
        }
        return parsed;
    }

    void MrsUavFcuApi::messageParser()
    {

        while (rclcpp::ok())
        {
            umsg_MessageToTransfer msg = ser_->waitForPacket();
            bool parsed = false;

            //RCLCPP_INFO(node_->get_logger(),"[HWApi] Received message class %d and type %d", msg.s.msg_class, msg.s.msg_type);
            if (_simulation_)
            {
                parsed = hitl_binder_.ParseMessage(msg) || ParseMessage(msg);
            }
            else
            {
                parsed = ParseMessage(msg);
            }

            if (!parsed)
            {
                RCLCPP_ERROR(node_->get_logger(),"[HWApi] message class %d and type %d could not be parsed", msg.s.msg_class, msg.s.msg_type);
            }
        }
    };

    // | ------------------------- methods ------------------------ |

    bool MrsUavFcuApi::callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        return false;
    }
    bool MrsUavFcuApi::callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        return false;
    }
    bool MrsUavFcuApi::callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        return false;
    }
    bool MrsUavFcuApi::callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        return false;
    }
    bool MrsUavFcuApi::callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        return false;
    }
    bool MrsUavFcuApi::callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        return false;
    }
    bool MrsUavFcuApi::callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;

        return false;
    }
    void MrsUavFcuApi::callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg)
    {
        /*To silence compiler*/
        (void)msg;
        
        return;
    }
    //}

} // namespace mrs_uav_fcu_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_fcu_api::MrsUavFcuApi, mrs_uav_hw_api::MrsUavHwApi)