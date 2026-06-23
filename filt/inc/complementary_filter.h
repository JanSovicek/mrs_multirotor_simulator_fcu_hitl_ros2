#pragma once

/*//{ includes */
/*//}*/
#include <Eigen/Geometry>
#include "umsg_sensors.h"
#include <string>

namespace attitude_estimation
{

/*//{ struct complementary_filter_params */
struct complementary_filter_params_t
{
  bool output_enu;

  bool debug_filter_steps;
  bool debug_gyro_bias;
  bool debug_mag_field;

  float ss_acc_threshold;            // for steady state estimation
  float ss_ang_vel_threshold;        // for steady state estimation
  float ss_delta_ang_vel_threshold;  // for steady state estimation

  bool  use_gyro_bias;
  float bias_alpha;  // alpha for gyro bias estimation

  bool  use_adaptive_gain;
  float default_accel_gain;  // default gain for accelerometer, needs to be kept below 0.1 due to small angle approximation in the correction quaternion

  float bottom_accel_error_threshold;  // (g) for adaptive gain
  float top_accel_error_threshold;     // (g) for adaptive gain

  float max_rotation_rate_cutoff;     // (rad/s) rotation rate at which fusing of accelerometer is disabled (to stop attitude deviation during flaring)

  bool  use_mag_correction;
  float mag_gain_p;
  float mag_gain_i;
  float mag_acc_threshold;
  float max_yaw_bias_rad_s; // maximum yaw bias in radians per second, to prevent excessive gyro bias
  float max_roll_pitch_bias_rad_s; // maximum roll/pitch bias in radians per second, to prevent excessive gyro bias
  float mag_timeout_s; // timeout for magnetometer data, if exceeded, mag correction is zeroed out to prevent runaway drift

  float max_dt;
};

/*//}*/

class ComplementaryFilter {

public:
  /*Sets complementary filter parameter and constrains*/
  ComplementaryFilter(const complementary_filter_params_t& params);
  /*Returns attitude quaternion in body to world rotation direction*/
  Eigen::Quaternion<float> getEstimation(bool* isValid);
  /*Updates IMU and iterates attitude estimate*/
  void                     updateImu(umsg_sensors_imu_t& imuMsg);
  /*Updates magnetometer*/
  void                     updateMag(umsg_sensors_mag_t& magMsg);
  Eigen::Vector3f          getGyroBias();
  void                     setGyroBias(Eigen::Vector3f bias);

private:
  std::string name_;

  bool is_initialized_          = false;
  bool is_first_iteration_      = true;
  bool is_using_mag_correction_ = false;
  bool mag_correction_ready_    = false;
  bool first_mag_data_          = true;

  uint64_t                 prev_stamp_IMU_ = 0;
  uint64_t                 prev_stamp_MAG_ = 0;
  Eigen::Quaternion<float> prev_attitude_;
  Eigen::Vector3f          prev_ang_vel_;
  Eigen::Vector3f          bias_ang_vel_;

  // MAGNETOMETER RELATED STUFF
  umsg_sensors_mag_t mag_msg_;
  Eigen::Vector3f mag_rate_correction_;

  Eigen::Quaternion<float> q_nwu_to_enu_;

  complementary_filter_params_t params_;

private:
  Eigen::Quaternion<float> getOrientationFromAccel(const Eigen::Vector3f& accel);
  Eigen::Quaternion<float> getOrientationFromMagField(const Eigen::Vector3f& mag_field);
  Eigen::Quaternion<float> getOrientationFromAccelAndMag(const Eigen::Vector3f& accel, const Eigen::Vector3f& mag_field);

  Eigen::Quaternion<float> iterateFilter(const Eigen::Vector3f& accel, const Eigen::Vector3f& ang_vel, const float dt, uint64_t current_timestamp);
  Eigen::Quaternion<float> predictOrientationFromGyro(const Eigen::Vector3f& ang_vel, const float dt);
  Eigen::Quaternion<float> predictOrientationFromGyroRK2(const Eigen::Vector3f& ang_vel, const Eigen::Vector3f& ang_vel_prev, const float dt);
  Eigen::Quaternion<float> correctionOrientationFromAccel(const Eigen::Vector3f& accel, const Eigen::Quaternion<float>& q_pred);
  float                    correctionOrientationFromMagField(const Eigen::Vector3f& mag_field, const Eigen::Quaternion<float>& q_corr_acc);
   
  bool isInSteadyState(const Eigen::Vector3f& accel, const Eigen::Vector3f& ang_vel);
  bool canFuseMag(const Eigen::Vector3f& accel);
  void updateGyroBias(const Eigen::Vector3f& ang_vel);

  float                    getAdaptiveGain(const Eigen::Vector3f& accel, const float default_gain);
  Eigen::Quaternion<float> scaleCorrection(const Eigen::Quaternion<float>& q_corr, const float scale);
  Eigen::Quaternion<float> nlerp(const Eigen::Quaternion<float>& q, const float scale);
  
  Eigen::Vector3f rotateVectorByQuaternion(const Eigen::Vector3f& vec, const Eigen::Quaternion<float>& q);

  
};

}  // namespace attitude_estimation