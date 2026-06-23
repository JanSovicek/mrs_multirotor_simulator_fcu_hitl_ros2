

#include "exp_forgetting_filter.hpp"


filters::exp_forgetting::exp_forgetting() {
}

void filters::exp_forgetting::init(float init_val, float lambda) {
  _lambda  = lambda;
  last_val = init_val;
}

float filters::exp_forgetting::step(float input_val) {
  float result = _lambda * input_val + (1 - _lambda) * last_val;
  last_val     = result;
  return result;
}