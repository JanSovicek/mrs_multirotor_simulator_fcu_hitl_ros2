#include "high_pass_filter.hpp"

namespace filters
{

HighPassFilter::HighPassFilter() {
    reset();
}

void HighPassFilter::reset() {
    for (int i = 0; i < 3; ++i) {
        d1_[i] = 0.0;
        d2_[i] = 0.0;
    }
}

double HighPassFilter::update(double input) {
    // Direct Form II Transposed Difference Equation for Channel 0
    const double output = b0_ * input + d1_[0];
    d1_[0] = b1_ * input - a1_ * output + d2_[0];
    d2_[0] = b2_ * input - a2_ * output;
    
    return output;
}

void HighPassFilter::update3D(double input_x, double input_y, double input_z,
                             double& out_x, double& out_y, double& out_z) {
    const double in[3] = {input_x, input_y, input_z};
    double out[3];

    for (int i = 0; i < 3; ++i) {
        out[i] = b0_ * in[i] + d1_[i];
        d1_[i] = b1_ * in[i] - a1_ * out[i] + d2_[i];
        d2_[i] = b2_ * in[i] - a2_ * out[i];
    }

    out_x = out[0];
    out_y = out[1];
    out_z = out[2];
}

}