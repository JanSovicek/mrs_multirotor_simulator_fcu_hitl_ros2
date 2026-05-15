#pragma once

//#include <cmsis_os.h>
#include <array>

namespace filters
{

//// Scaled for floating point //0.022117332824280193, -0.02881132484364853, 0.022117332824280196, 1.8184380919027663,  -0.8338614327076781  // b0, b1, b2, a1,
/// a2

typedef enum {
  FILTER_AXIS_X = 0,
  FILTER_AXIS_Y = 1,
  FILTER_AXIS_Z = 2
} Filters_Axis_e;

class lowpass_filter {
private:
  float incoming_weight = 1.0;
  float weights_in[2]   = {0, 0};
  float weights_out[2]  = {0, 0};
  float buffer_in[2]    = {0, 0};
  float buffer_out[2]   = {0, 0};

public:
  void init(float val);
  lowpass_filter(float b_arr[3], float a_arr[3]);
  lowpass_filter();
  float step(float data_in);
};

class K_filter {
private:
  lowpass_filter stage1;
  lowpass_filter stage2;
  /* data */
public:
  K_filter();
  ~K_filter();
  void  init(float val);
  float step(float val);
};

class lowpass_filter3d {
private:
  float incoming_weight = 1.0;
  float weights_in[2]   = {0, 0};
  float weights_out[2]  = {0, 0};

  float buffer_0_in[2]  = {0, 0};
  float buffer_0_out[2] = {0, 0};

  float buffer_1_in[2]  = {0, 0};
  float buffer_1_out[2] = {0, 0};

  float buffer_2_in[2]  = {0, 0};
  float buffer_2_out[2] = {0, 0};
  float filter_axis(float data, Filters_Axis_e eAxis);

public:
  void init(float val_x, float val_y, float val_z);
  lowpass_filter3d(float b_arr[3], float a_arr[3]);
  lowpass_filter3d();
  std::array<float, 3> step(std::array<float, 3> data_in);
};


class acceleration_filter 
{
private:
  lowpass_filter3d _lowpass_filter_3D;

public:
  acceleration_filter();
  void init(float val_x, float val_y, float val_z);
  void step(float data_in_out[3]);
};

class gyro_filter
{
private:
  lowpass_filter3d _lowpass_filter_3D;
public:
  gyro_filter();
  void init(float val_x, float val_y, float val_z);
  std::array<float, 3> step(std::array<float, 3> data_in);
};

}  // namespace filters
