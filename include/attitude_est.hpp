#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <umsg_sensors.h>       
#include "complementary_filter.h"
#include "low_pass_filter.hpp"

class AttitudeEstimator
{
private:
    attitude_estimation::ComplementaryFilter *p_attitude_filter_;
    filters::acceleration_filter *p_acc_filt_;
    // Declare private member variables and methods here
public:
    void Init();
    void UpdateMag(const umsg_sensors_mag_t& magMsg);
    void UpdateImu(const umsg_sensors_imu_t& imuMsg);
    Eigen::Quaternion<float> GetEstimation(bool* isValid);
};