#include "attitude_est.hpp"

void AttitudeEstimator::Init() {

        /*Create complementary filter*/
        attitude_estimation::complementary_filter_params_t params;

        params.output_enu         = true;  // outputs in east north up frame
        params.debug_filter_steps = false;
        params.debug_gyro_bias    = false;
        params.debug_mag_field    = false;

        params.ss_acc_threshold           = 5e-2;
        params.ss_ang_vel_threshold       = 5e-2 * M_PI;
        params.ss_delta_ang_vel_threshold = 1e-5 * M_PI;

        params.use_gyro_bias = false;
        params.bias_alpha    = 1e-4;

        params.use_adaptive_gain  = true;
        params.default_accel_gain = 0.003; // value between 0.001 and 0.005, lower value cuts to much accelerometer, higher allows too much noice from accelerometer

        params.bottom_accel_error_threshold = 0.15; //0.05 // below this error, the full accel correction gain is applied
        params.top_accel_error_threshold    = 0.4; // above this error, the accel correction gain is zero, between bottom and top, the gain is scaled linearly

        params.use_mag_correction = true;
        params.mag_gain_p         = 0.1; // Proportional gain for magnetometer correction
        params.mag_gain_i         = 1e-3; // Integral gain for magnetometer correction
        params.mag_acc_threshold  = 0.15;

        params.max_dt             = 0.1;  // Maximum allowed time step for prediction, in seconds
            
        params.max_yaw_bias_rad_s = 0.05; // MEMS gyros rarely drift more than a few degrees per second. If the bias exceeds ~0.05 rad/s (approx 3 deg/s), it is magnetic EMI, not gyro drift.

        p_attitude_filter_ = new attitude_estimation::ComplementaryFilter(params);

        /*Create acceleration filter*/
        p_acc_filt_ = new filters::acceleration_filter();
 
        return;
}

void AttitudeEstimator::UpdateImu(const umsg_sensors_imu_t& imuMsg) {
    p_acc_filt_->step(imuMsg.accel);
    p_attitude_filter_->updateImu(imuMsg);
}

void AttitudeEstimator::UpdateMag(const umsg_sensors_mag_t& magMsg) {
    p_attitude_filter_->updateMag(magMsg);
}

Eigen::Quaternion<float> AttitudeEstimator::GetEstimation(bool* isValid) {
    return p_attitude_filter_->getEstimation(isValid);
}
