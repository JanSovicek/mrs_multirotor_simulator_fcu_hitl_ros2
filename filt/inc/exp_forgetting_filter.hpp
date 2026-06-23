//#include "cmsis_os.h"

#pragma once


namespace filters
{

class exp_forgetting {
private:
  float last_val = 0;
  float _lambda  = 0;

public:
  exp_forgetting();
  void  init(float init_val, float lambda);
  float step(float input_val);
};


}  // namespace filters