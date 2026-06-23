
#include "low_pass_filter.hpp"

using namespace filters;


lowpass_filter::lowpass_filter() {}


lowpass_filter::lowpass_filter(float b_arr[3], float a_arr[3]) {
  incoming_weight = b_arr[0];
  weights_in[0]   = b_arr[1];
  weights_in[1]   = b_arr[2];

  weights_out[0] = a_arr[1];
  weights_out[1] = a_arr[2];
}

void lowpass_filter::init(float val) {
  buffer_in[0]  = val;
  buffer_in[1]  = val;
  buffer_out[0] = val;
  buffer_out[1] = val;
}


float lowpass_filter::step(float data_in) {
  float output =
      data_in * incoming_weight + buffer_in[0] * weights_in[0] + buffer_in[1] * weights_in[1] + buffer_out[0] * weights_out[0] + buffer_out[1] * weights_out[1];
  // shift the data
  buffer_in[1]  = buffer_in[0];
  buffer_in[0]  = data_in;
  buffer_out[1] = buffer_out[0];
  buffer_out[0] = output;
  return output;
}


K_filter::K_filter() {

  /*
  // Scaled for floating point

  0.00023753210861417686, 0.0004750642172283537, 0.00023753210861417686, 1.9438749286031587, -0.9447309071611426,// b0, b1, b2, a1, a2
  0.000244140625, 0.00048828125, 0.000244140625, 1.9584652745812048, -0.9595492500974367// b0, b1, b2, a1, a2


*/
  float b_arr_1[] = {0.00023753210861417686, 0.0004750642172283537, 0.00023753210861417686};
  float a_arr_1[] = {1, 1.9438749286031587, -0.9447309071611426};


  float b_arr_2[] = {0.000244140625, 0.00048828125, 0.000244140625};
  float a_arr_2[] = {1, 1.9584652745812048, -0.9595492500974367};


  stage1 = lowpass_filter(b_arr_1, a_arr_1);
  stage2 = lowpass_filter(b_arr_2, a_arr_2);
}
K_filter::~K_filter() {
}
void K_filter::init(float val) {
  stage1.init(val);
  stage2.init(val);
}
float K_filter::step(float val) {
  return stage2.step(stage1.step(val));
}


lowpass_filter3d::lowpass_filter3d(float b_arr[3], float a_arr[3]) {
  incoming_weight = b_arr[0];
  weights_in[0]   = b_arr[1];
  weights_in[1]   = b_arr[2];

  weights_out[0] = a_arr[1];
  weights_out[1] = a_arr[2];
}


float lowpass_filter3d::filter_axis(float acceleration_data, Filters_Axis_e eAxis) {
  float *buffer_in = nullptr;
  float *buffer_out = nullptr;
  float output = acceleration_data;

  switch (eAxis) {
    case FILTER_AXIS_X:
      buffer_in  = buffer_0_in;
      buffer_out = buffer_0_out;
      /* code */
      break;
    case FILTER_AXIS_Y:
      buffer_in  = buffer_1_in;
      buffer_out = buffer_1_out;
      /* code */
      break;
    case FILTER_AXIS_Z:
      buffer_in  = buffer_2_in;
      buffer_out = buffer_2_out;
      /* code */
      break;
    default:
      /*Invalid input*/
      break;
  }

  if((buffer_in != nullptr) && (buffer_out != nullptr)) 
  {
    // Adhere to the standard DSP difference equation!
    output =  acceleration_data  * incoming_weight 
                 + buffer_in[0]  * weights_in[0] 
                 + buffer_in[1]  * weights_in[1] 
                 - buffer_out[0] * weights_out[0] 
                 - buffer_out[1] * weights_out[1];
                  
    // shift the data
    buffer_in[1]  = buffer_in[0];
    buffer_in[0]  = acceleration_data;
    buffer_out[1] = buffer_out[0];
    buffer_out[0] = output;
  }

  return output;
}

void lowpass_filter3d::init(float val_x, float val_y, float val_z) {
  buffer_0_in[0]  = val_x;
  buffer_0_in[1]  = val_x;
  buffer_0_out[0] = val_x;
  buffer_0_out[1] = val_x;

  buffer_1_in[0]  = val_y;
  buffer_1_in[1]  = val_y;
  buffer_1_out[0] = val_y;
  buffer_1_out[1] = val_y;

  buffer_2_in[0]  = val_z;
  buffer_2_in[1]  = val_z;
  buffer_2_out[0] = val_z;
  buffer_2_out[1] = val_z;
}


std::array<float, 3> lowpass_filter3d::step(std::array<float, 3> data_in) 
{
  std::array<float, 3> data_out;
  data_out[0] = filter_axis(data_in[0], FILTER_AXIS_X);
  data_out[1] = filter_axis(data_in[1], FILTER_AXIS_Y);
  data_out[2] = filter_axis(data_in[2], FILTER_AXIS_Z);
  
  return data_out;
}

lowpass_filter3d::lowpass_filter3d() {

}


acceleration_filter::acceleration_filter() {
  // Scaled for floating point

  //    0.04539244326738814, -0.07011937165653818, 0.045392443267388144, 1.7907660630536484, -0.8114315779318865// b0, b1, b2, a1, a2

  /*
   no glue spectrum - not rdy
  float b_arr_1[] = {0.04539244326738814, -0.07011937165653818, 0.045392443267388144};
  float a_arr_1[] = {1, 1.7907660630536484, -0.8114315779318865};
  */
  /*
  // sharp filred 40dB

      0.09526700266189454, -0.08505007961963645, 0.09526700266189454, 1.551652005607951, -0.6106832010529782,// b0, b1, b2, a1, a2
      0.125, -0.219187481574096, 0.125, 1.794530174737618, -0.849589628978599// b0, b1, b2, a1, a2
  */

  /*// Scaled for floating point

    0.20559061829647124, -0.06668747948750167, 0.20559061829647124, 1.156644923759876, -0.3669878197632643,// b0, b1, b2, a1, a2
    0.25, -0.3899281355833633, 0.25, 1.595009096156979, -0.7752817407053217// b0, b1, b2, a1, a2



  */

  // The original Michal Reiser's SOS (Second-Order Sections) filter, adapted for standard DSP math, causes Non-Linear Phase Delay based on the input event frequency - bad for PID gains
  //float b_arr_1[] = {0.2055906f, -0.0666874f, 0.2055906f};
  //float a_arr_1[] = {1.0f, -1.1566449f, 0.3669878f}; // Signs flipped!
  //
  //float b_arr_2[] = {0.25f, -0.3899281f, 0.25f};
  //float a_arr_2[] = {1.0f, -1.5950090f, 0.7752817f}; // Signs flipped!

  //stage1          = lowpass_filter3d(b_arr_1, a_arr_1);
  //stage2          = lowpass_filter3d(b_arr_2, a_arr_2);


  // 30Hz 2nd-Order Butterworth (1000Hz Sample Rate) - designed for a maximally flat magnitude response in the passband
  // Standard format: a_0 is assumed to be 1.0 and is omitted from the math
  //float b_arr_flight[] = {0.007820208033497193f, 0.015640416066994386f, 0.007820208033497193f};
  //float a_arr_flight[] = {1.0f, -1.734725768809275f, 0.7660066009432638f};

  // 2nd Order Butterworth Lowpass Filter @ 500Hz Sampling, 30Hz Cutoff
  float b_arr_flight[3] = {0.02785977f, 0.05571953f, 0.02785977f};
  float a_arr_flight[3] = {1.0f, -1.47548044f, 0.58691951f};

  _lowpass_filter_3D = lowpass_filter3d(b_arr_flight, a_arr_flight);
}

void acceleration_filter::init(float val_x, float val_y, float val_z) {
  _lowpass_filter_3D.init(val_x, val_y, val_z);
}

void acceleration_filter::step(float data_in_out[3]) 
{
  std::array<float, 3> data = {data_in_out[0], data_in_out[1], data_in_out[2]};

  data = _lowpass_filter_3D.step(data);

  data_in_out[0] = data[0];
  data_in_out[1] = data[1];
  data_in_out[2] = data[2];

  return;
}


gyro_filter::gyro_filter() {

  //PT1 filter - mathematically equivalent to a first-order low-pass Butterworth filter
  //Designed for a cutoff frequency of 100Hz and a sample rate of 1000Hz, providing a good balance between noise reduction and responsiveness for gyroscopic data in attitude control
  float b_arr_flight[] = {0.24523727525278557f, 0.24523727525278557f, 0.0f};
  float a_arr_flight[] = {1.0f, -0.5095254494944288f, 0.0f};

  _lowpass_filter_3D = lowpass_filter3d(b_arr_flight, a_arr_flight);
}

void gyro_filter::init(float val_x, float val_y, float val_z) {
  _lowpass_filter_3D.init(val_x, val_y, val_z);
}

std::array<float, 3> gyro_filter::step(std::array<float, 3> data_in) 
{
  std::array<float, 3> data_out;

  data_out = _lowpass_filter_3D.step(data_in);

  return data_out;
}
