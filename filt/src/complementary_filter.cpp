#include "complementary_filter.h"
#include <sys/types.h>
#include <cmath>

namespace attitude_estimation
{


/*//{ constructor */
ComplementaryFilter::ComplementaryFilter(const complementary_filter_params_t& params) {

  q_nwu_to_enu_         = Eigen::Quaternion<float>(1, 0, 0, 1).normalized();  // transformation of frame from NWU to ENU - that is equivalent to a rotation of the vector by 90 degrees around the Z axis
  is_initialized_       = true;
  params_               = params;
  bias_ang_vel_         = bias_ang_vel_.Zero();
  prev_attitude_        = Eigen::Quaternion<float>(1, 0, 0, 0);
  mag_rate_correction_  = Eigen::Vector3f::Zero();
}
/*//}*/

Eigen::Quaternion<float> ComplementaryFilter::getEstimation(bool* isValid) {
  // if the first iteration did still not go throught, the position is not valid
  *isValid = !is_first_iteration_;
  return prev_attitude_;
}

/*//{ callbackImu() */
void ComplementaryFilter::updateImu(umsg_sensors_imu_t& imuMsg) {
  if (!is_initialized_) {
    return;
  }

  // initialize in first iteration
  if (is_first_iteration_) {

    Eigen::Vector3f accel, ang_vel;
    accel = Eigen::Vector3f(imuMsg.accel);

    if (params_.use_mag_correction) {

      if (!mag_correction_ready_) {
        return;
      }
      // MAG STUFF
      Eigen::Vector3f mag_field(mag_msg_.mag);
      prev_attitude_ = getOrientationFromAccelAndMag(accel, mag_field);

      is_using_mag_correction_ = true;

    } else {
      prev_attitude_ = getOrientationFromAccel(accel);
    }
    ang_vel             = Eigen::Vector3f(imuMsg.gyro);
    prev_ang_vel_       = ang_vel;
    prev_stamp_IMU_     = imuMsg.timestamp;
    is_first_iteration_ = false;

    return;
  }

  Eigen::Vector3f accel, ang_vel;

  accel   = Eigen::Vector3f(imuMsg.accel);
  ang_vel = Eigen::Vector3f(imuMsg.gyro);


  if (imuMsg.timestamp >= prev_stamp_IMU_) 
  {
    float dt = static_cast<float>(imuMsg.timestamp - prev_stamp_IMU_)*1e-6; // Convert microseconds to seconds
    // main filter loop
    const Eigen::Quaternion<float> attitude_filtered = iterateFilter(accel, ang_vel, dt, imuMsg.timestamp);
    prev_attitude_                                   = attitude_filtered;
    // prepare for next iteration
    prev_stamp_IMU_                                  = imuMsg.timestamp;
    prev_ang_vel_                                    = ang_vel;
  }
}
/*//}*/

/*//{ callbackMag() */
void ComplementaryFilter::updateMag(umsg_sensors_mag_t& magMsg) {
  
  if (first_mag_data_) {
    prev_stamp_MAG_ = magMsg.timestamp;
    mag_msg_        = magMsg;
    first_mag_data_ = false;
    mag_correction_ready_ = true;
  }
  else if((params_.max_dt >= static_cast<float>(magMsg.timestamp - prev_stamp_MAG_) * 1e-6) && (magMsg.timestamp > prev_stamp_MAG_))
  {
    prev_stamp_MAG_ = mag_msg_.timestamp;
    mag_msg_        = magMsg;
    mag_correction_ready_ = true;
  }
}
/*//}*/

/*//{ getGyroBias()*/
Eigen::Vector3f ComplementaryFilter::getGyroBias() {
  return bias_ang_vel_;
}
/*//}*/

/*//{ setGyroBias()*/
void ComplementaryFilter::setGyroBias(Eigen::Vector3f bias) {
  if (params_.use_gyro_bias) {
    bias_ang_vel_ = bias;
  }
}
/*//}*/

/*//{ getOrientationFromAccel() */
Eigen::Quaternion<float> ComplementaryFilter::getOrientationFromAccel(const Eigen::Vector3f& accel) {

  const Eigen::Vector3f    a_norm = accel.normalized();
  Eigen::Quaternion<float> q;

  if (a_norm.z() >= 0) {

    // singularity at az=-1
    float qw = sqrtf((a_norm.z() + 1) * 0.5);
    q.x()    = (-a_norm.y() / (2.0 * qw));
    q.y()    = (a_norm.x() / (2.0 * qw));
    q.z()    = 0;
    q.w()    = qw;

  } else {

    // singularity at az=1
    float qx = sqrtf((1 - a_norm.z()) * 0.5);
    q.x()    = (qx);
    q.y()    = (0);
    q.z()    = (a_norm.x() / (2.0 * qx));
    q.w()    = (-a_norm.y() / (2.0 * qx));
  }

  return q.normalized().conjugate();
}
/*//}*/

/*//{ getOrientationFromMagField() */
Eigen::Quaternion<float> ComplementaryFilter::getOrientationFromMagField(const Eigen::Vector3f& mag_field) {

  const Eigen::Vector3f    l_norm = mag_field;
  Eigen::Quaternion<float> q;

  float gamma = l_norm.x() * l_norm.x() + l_norm.y() * l_norm.y();

  if (l_norm.x() >= 0) {
    q.x() = 0;
    q.y() = 0;
    q.z() = l_norm.y() / (sqrtf(2) * sqrtf(gamma + l_norm.x() * sqrtf(gamma)));
    q.w() = (sqrtf(gamma + l_norm.x() * sqrtf(gamma))) / sqrtf(2 * gamma);
  } else {
    q.x() = (0);
    q.y() = (0);
    q.z() = ((sqrtf(gamma - l_norm.x() * sqrtf(gamma))) / sqrtf(2 * gamma));
    q.w() = (l_norm.y() / (sqrtf(2) * sqrtf(gamma - l_norm.x() * sqrtf(gamma))));  
  }

  return q.normalized();
}
/*//}*/

/*//{ getOrientationFromAccelAndMag() */
Eigen::Quaternion<float> ComplementaryFilter::getOrientationFromAccelAndMag(const Eigen::Vector3f& accel, const Eigen::Vector3f& mag_field) {

  // obtain tilt from accelerometer
  Eigen::Quaternion<float> q_tilt_b2w = getOrientationFromAccel(accel);

  // rotate magnetic field vector to world frame
  Eigen::Vector3f mag_field_leveled = rotateVectorByQuaternion(mag_field, q_tilt_b2w);

  // check if the horizontal component of the magnetic field is strong enough for a reliable heading estimate
  float horizontal_mag = sqrtf(mag_field_leveled.x() * mag_field_leveled.x() + 
                             mag_field_leveled.y() * mag_field_leveled.y());

  if (horizontal_mag < 0.1f) {
   // DOTO: Warning: Magnetic field is too vertical or weak to find a heading!
  }

  // get yaw correction from magnetometer
  float initial_yaw = 0.0f;
  if(params_.output_enu) {
    initial_yaw = atan2f(mag_field_leveled.x(), mag_field_leveled.y());
  } else {
    initial_yaw = atan2f(-mag_field_leveled.y(), mag_field_leveled.x());
  }

  // create the yaw-only quaternion
  Eigen::Quaternion<float> q_yaw_b2w(Eigen::AngleAxisf(initial_yaw, Eigen::Vector3f::UnitZ()));

  // combine tilt and yaw to get the initial attitude estimate
  Eigen::Quaternion<float> q_b2w = q_yaw_b2w * q_tilt_b2w; 

  return q_b2w.normalized();
}
/*//}*/


/*//{ iterateFilter() */
Eigen::Quaternion<float> ComplementaryFilter::iterateFilter(const Eigen::Vector3f& accel, const Eigen::Vector3f& ang_vel, const float dt, uint64_t current_timestamp_IMU) {

  // guard against impossible dt
  if (dt <= 0.0 || dt > params_.max_dt) {
      return prev_attitude_;
  }

  // estimate bias only in steady state
  if (params_.use_gyro_bias && isInSteadyState(accel, ang_vel) && (params_.use_mag_correction == false)) {
    updateGyroBias(ang_vel);
  }

  // calculate the corrected angular velocity
  // ang_vel: raw IMU (1000Hz)
  // bias_ang_vel_: slow drifting integral bias (updated at 50Hz/1000Hz)
  // mag_rate_correction_z_: the P-term nudge towards North (updated at 50Hz)
  Eigen::Vector3f corrected_ang_vel = ang_vel - bias_ang_vel_;

  if(params_.use_mag_correction)
  {
    corrected_ang_vel += mag_rate_correction_; 
    // Decay the proportional Mag correction to zero over time
    mag_rate_correction_ *= 0.9f;
  }

  // predict orientation using unbiased gyro measurements
  const Eigen::Quaternion<float> q_pred = predictOrientationFromGyro(corrected_ang_vel, dt);

  // obtain orientation correction in the form of delta quaternion from measured acceleration
  const Eigen::Quaternion<float> dq_corr_acc = correctionOrientationFromAccel(accel, q_pred);


  // estimate gain based on acceleration magnitude
  float accel_gain;
  if (params_.use_adaptive_gain) {
    accel_gain = getAdaptiveGain(accel, params_.default_accel_gain);
  } else {
    accel_gain = params_.default_accel_gain;
  }

  // scale correction by the accel_gain to reduce influence of high-frequency noise
  const Eigen::Quaternion<float> dq_corr_acc_sc = scaleCorrection(dq_corr_acc, accel_gain);

  // combine prediction with acceleration correction
  Eigen::Quaternion<float> q_corrected = q_pred * dq_corr_acc_sc;

  // apply magnetic field correction if it is ready
  if (params_.use_mag_correction && mag_correction_ready_ && canFuseMag(accel)) {

    mag_correction_ready_ = false;

    // time sanity check
    if((params_.max_dt >= static_cast<float>(current_timestamp_IMU - mag_msg_.timestamp) * 1e-6) && (mag_msg_.timestamp > prev_stamp_MAG_)) 
    {
      Eigen::Vector3f mag_field(mag_msg_.mag[0], mag_msg_.mag[1], mag_msg_.mag[2]);

      // obtain orientation correction in the form of delta quaternion from measured magnetic field
      float yaw_error_rad = correctionOrientationFromMagField(mag_field, q_corrected);

      // ensure the error is strictly bounded between -PI and PI
      if (yaw_error_rad > static_cast<float>(M_PI))  yaw_error_rad -= 2.0f * static_cast<float>(M_PI);
      if (yaw_error_rad < -static_cast<float>(M_PI)) yaw_error_rad += 2.0f * static_cast<float>(M_PI);

      // update the gyro bias (The INTEGRAL Term)
      // use dt_mag (e.g., ~0.02s) at 50Hz.
      // params_.mag_gain_i should be very small (e.g., 0.001)
      float dt_mag = static_cast<float>(mag_msg_.timestamp - prev_stamp_MAG_) * 1e-6;
      float total_correction = (params_.mag_gain_i * yaw_error_rad * dt_mag);

      // rotate gravity vector into body frame
      const Eigen::Vector3f gravity_body = rotateVectorByQuaternion(Eigen::Vector3f::UnitZ(), q_corrected.conjugate());

      // project the correction into the local body frame axes 
      Eigen::Vector3f projected_correction = gravity_body * total_correction;
      bias_ang_vel_ += projected_correction;

      // anti-Windup Clamp
      if (bias_ang_vel_.x() > params_.max_yaw_bias_rad_s)  bias_ang_vel_.x() = params_.max_roll_pitch_bias_rad_s;
      if (bias_ang_vel_.x() < -params_.max_yaw_bias_rad_s) bias_ang_vel_.x() = -params_.max_roll_pitch_bias_rad_s;

      // anti-Windup Clamp
      if (bias_ang_vel_.y() > params_.max_yaw_bias_rad_s)  bias_ang_vel_.y() = params_.max_roll_pitch_bias_rad_s;
      if (bias_ang_vel_.y() < -params_.max_yaw_bias_rad_s) bias_ang_vel_.y() = -params_.max_roll_pitch_bias_rad_s;

      // anti-Windup Clamp
      if (bias_ang_vel_.z() > params_.max_yaw_bias_rad_s)  bias_ang_vel_.z() = params_.max_yaw_bias_rad_s;
      if (bias_ang_vel_.z() < -params_.max_yaw_bias_rad_s) bias_ang_vel_.z() = -params_.max_yaw_bias_rad_s;

      // calculate the instantaneous rate correction (the PROPORTIONAL term)
      // this is stored in a class member variable (e.g., mag_rate_correction_z_)
      // so it can be applied continuously at 1000Hz IMU loop rate
      mag_rate_correction_ = gravity_body * (params_.mag_gain_p * yaw_error_rad);
    }
  } 

  return q_corrected.normalized();
}
/*//}*/

/*//{ predictOrientationFromGyro()*/
Eigen::Quaternion<float> ComplementaryFilter::predictOrientationFromGyro(const Eigen::Vector3f& ang_vel, const float dt) {

  // time sanity check
  if (dt <= 0 || dt > params_.max_dt) {
    return prev_attitude_;
  }

  // if angular velocity is effectively zero, skip prediction step, return previous attitude
  if (ang_vel.squaredNorm() < 1e-12) {
    return prev_attitude_;
  }

  // construct pure quaternion from angular velocity
  const Eigen::Quaternion<float> q_ang_vel_pure(0, ang_vel.x(), ang_vel.y(), ang_vel.z());

  // get quaternion derivative from angular velocity
  Eigen::Quaternion<float> dq_pred;
  dq_pred.coeffs() = (prev_attitude_ * q_ang_vel_pure).coeffs() * 0.5;

  // integrate prediction
  Eigen::Quaternion<float> q_pred;
  q_pred.coeffs() = prev_attitude_.coeffs() + (dq_pred.coeffs() * dt);

  return q_pred;
}
/*//}*/

/*//{ correctionOrientationFromAccel() */
Eigen::Quaternion<float> ComplementaryFilter::correctionOrientationFromAccel(const Eigen::Vector3f& accel, const Eigen::Quaternion<float>& q_pred) {

  //rotate gravity vector into body frame
  const Eigen::Vector3f gravity_body = rotateVectorByQuaternion(Eigen::Vector3f::UnitZ(), q_pred.conjugate());

  // calculate the error between measured and expected gravity vector, this is the axis of rotation needed to correct the attitude
  Eigen::Vector3f error = accel.normalized().cross(gravity_body);
  
  // build the correction quaternion from the error vector, 
  // use small angle approximation (sin(theta/2) ~ theta/2) since the correction is expected to be small between iterations
  const Eigen::Quaternion<float> dq_corr(1, error.x()*0.5, error.y()*0.5, error.z()*0.5);

  return dq_corr.normalized();
}
/*//}*/

/*//{ correctionOrientationFromMagField() */
float ComplementaryFilter::correctionOrientationFromMagField(const Eigen::Vector3f& mag_field, const Eigen::Quaternion<float>& q_corr_acc) {

  // guard against dead/corrupted magnetometer data
  float mag_norm = mag_field.norm();
  if (mag_norm < 1e-4f || !std::isfinite(mag_norm)) 
  {
    return 0.0f;
  }

  // global magnetic filed vector in NWU frame
  Eigen::Vector3f mag_field_global = Eigen::Vector3f::UnitX(); // ignoring vertical component

  // rotate global magnetic field vector to ENU frame if needed
  if (params_.output_enu) {
    mag_field_global = rotateVectorByQuaternion(mag_field_global, q_nwu_to_enu_);
  }

  // rotate magnetic field vector from world frame to body frame using the acceleration-corrected attitude estimate
  const Eigen::Vector3f mag_field_body_est = rotateVectorByQuaternion(mag_field_global, q_corr_acc.conjugate());

  // calculate the error between measured and expected magnetic field, this is the axis of rotation needed to correct the attitude
  Eigen::Vector3f error_3d = mag_field.normalized().cross(mag_field_body_est);

  // project the world vertical axis into the body frame
  Eigen::Vector3f gravity_body = rotateVectorByQuaternion(Eigen::Vector3f::UnitZ(), q_corr_acc.conjugate());

  // heading is the dot product of the 3D error vector and the gravity vector in the body frame
  float yaw_error = error_3d.dot(gravity_body);

  return yaw_error;
}

/*//}*/

/*//{ isInSteadyState() */
bool ComplementaryFilter::isInSteadyState(const Eigen::Vector3f& accel, const Eigen::Vector3f& ang_vel) {

  // acceleration too large
  if (std::abs(accel.norm() - 1) > params_.ss_acc_threshold) {
    return false;
  }

  // angular velocity too large
  if (fabs(ang_vel.x() - bias_ang_vel_.x()) > params_.ss_ang_vel_threshold || fabs(ang_vel.y() - bias_ang_vel_.y()) > params_.ss_ang_vel_threshold ||
      fabs(ang_vel.z() - bias_ang_vel_.z()) > params_.ss_ang_vel_threshold) {
    return false;
  }

  // delta angular velocity too large
  if (fabs(ang_vel.x() - prev_ang_vel_.x()) > params_.ss_delta_ang_vel_threshold ||
      fabs(ang_vel.y() - prev_ang_vel_.y()) > params_.ss_delta_ang_vel_threshold ||
      fabs(ang_vel.z() - prev_ang_vel_.z()) > params_.ss_delta_ang_vel_threshold) {
    return false;
  }

  return true;
}
/*//}*/

/*//{ canFusedMag() */
bool ComplementaryFilter::canFuseMag(const Eigen::Vector3f& accel) {

  // acceleration too large
  if (std::abs(accel.norm() - 1) > params_.mag_acc_threshold) {
    return false;
  }

  return true;
}
/*//}*/

/*//{ updateGyroBias() */
void ComplementaryFilter::updateGyroBias(const Eigen::Vector3f& ang_vel) {

  // Angular velocity bias is only updated for X and Y axis, bias in Z axis is handled separately by the magnetometer correction
  bias_ang_vel_[0] = (bias_ang_vel_.x() + params_.bias_alpha * (ang_vel.x() - bias_ang_vel_.x()));
  bias_ang_vel_[1] = (bias_ang_vel_.y() + params_.bias_alpha * (ang_vel.y() - bias_ang_vel_.y()));
  bias_ang_vel_[2] = (bias_ang_vel_.z() + params_.bias_alpha * (ang_vel.z() - bias_ang_vel_.z()));
}
/*//}*/

/*//{ getAdaptiveGain() */
float ComplementaryFilter::getAdaptiveGain(const Eigen::Vector3f& accel, const float default_gain) {

  const float accel_error = std::abs(accel.norm() - 1) / 1;

  const float thr_bot = params_.bottom_accel_error_threshold;
  const float thr_top = params_.top_accel_error_threshold;

  float factor;
  if (accel_error < thr_bot) {
    factor = 1.0;

  } else if (accel_error > thr_top) {
    factor = 0.0;

    // linear decrease of gain factor with the error magnitude
  } else {
    float m = 1 / (thr_bot - thr_top);
    float b = -thr_top / (thr_bot - thr_top);
    factor  = m * accel_error + b;
  }

  return factor * default_gain;
}
/*//}*/

/*//{ scaleCorrection() */
Eigen::Quaternion<float> ComplementaryFilter::scaleCorrection(const Eigen::Quaternion<float>& q_corr, const float scale) {

  Eigen::Quaternion<float> q_scaled;
  q_scaled = nlerp(q_corr, scale);

  return q_scaled;
}
/*//}*/

/*//{ nlerp() */
Eigen::Quaternion<float> ComplementaryFilter::nlerp(const Eigen::Quaternion<float>& q, const float scale) {

  Eigen::Quaternion<float> q_interp;
  // nlerp: q_interp = (1 - scale)*Identity + scale*q
  // since identity is [w=1, x=0, y=0, z=0], this simplifies to:
  q_interp.w() = 1.0f + scale * (q.w() - 1.0f);
  q_interp.vec() = scale * q.vec();

  return q_interp.normalized();
}
/*//}*/

/*//{ rotateVectorByQuaternion<float>() */
Eigen::Vector3f ComplementaryFilter::rotateVectorByQuaternion(const Eigen::Vector3f& vec, const Eigen::Quaternion<float>& q) {
  // eigen's operator* for (quaternion * vector) is highly optimized.
  // it is mathematically equivalent to q * v * q.conjugate(), 
  // but uses roughly 50% fewer multiplications.
  return q*vec;
}
/*//}*/

}  // namespace attitude_estimation
