#pragma once

//#include <cstdint>

namespace filters
{

/**
 * @brief 2nd-Order High-Pass Butterworth Filter
 * 
 * Configured for f_cutoff = 80 Hz @ f_sample = 500 Hz (HITL Simulation Rate).
 * Uses Direct Form II Transposed structure executed in double precision
 * to prevent numerical accumulation errors.
 */
class HighPassFilter {
public:
    HighPassFilter();

    /**
     * @brief Process a single scalar sample through the filter.
     * @param input Raw signal sample (e.g., raw acceleration sample in m/s^2)
     * @return High-pass filtered sample (only frequencies > 80 Hz pass through)
     */
    double update(double input);

    /**
     * @brief Process a 3-axis vector sample (e.g., Vector3d IMU sample)
     * @param input_x X-axis sample
     * @param input_y Y-axis sample
     * @param input_z Z-axis sample
     * @param out_x High-pass filtered X output
     * @param out_y High-pass filtered Y output
     * @param out_z High-pass filtered Z output
     */
    void update3D(double input_x, double input_y, double input_z,
                  double& out_x, double& out_y, double& out_z);

    /**
     * @brief Reset internal filter states to zero.
     */
    void reset();

private:
    // Pre-calculated Biquad High-Pass Coefficients (fc = 80 Hz, fs = 500 Hz)
    static constexpr double b0_ =  0.51859550;
    static constexpr double b1_ = -1.03719101;
    static constexpr double b2_ =  0.51859550;
    
    static constexpr double a1_ = -0.80900010;
    static constexpr double a2_ =  0.26538192;

    // Filter state delays for 3 independent channels (X, Y, Z)
    double d1_[3]{0.0, 0.0, 0.0};
    double d2_[3]{0.0, 0.0, 0.0};
};

}