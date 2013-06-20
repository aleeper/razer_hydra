/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

#ifndef HYDRA_H
#define HYDRA_H

#include <tf/tf.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/hidraw.h>
#include <usb.h>

template <class T>
class Filter {
  virtual void zero() = 0;
  inline virtual const T& process(const T& x) = 0;
  virtual void setFc(double Fc, double Fs) = 0;
  inline virtual const T& getLast() { return y0; }

protected:
  T y0;
};

template <class T>
class OnePole : public Filter<T> { // http://www.earlevel.com/main/2012/12/15/a-one-pole-filter/
public:
  OnePole() : a0(0), b1(0) { }

  OnePole(double Fc, double Fs) : a0(0), b1(0) { setFc(Fc, Fs); }

  virtual void setFc(double Fc, double Fs)
  {
    b1 = exp(-2.0 * M_PI * Fc / Fs);
    a0 = 1.0 - b1;
  }

  inline virtual const T& process(const T& x) {
    //this->y0 = a0*x + b1*this->y0;
    return this->y0;
  }

protected:
    double a0, b1;
};

class OnePoleQuaternion : public OnePole<tf::Quaternion> {
public:
  OnePoleQuaternion() { zero(); }
  OnePoleQuaternion(double Fc, double Fs) : OnePole<tf::Quaternion>(Fc, Fs) { zero(); }

  virtual void zero() { y0 = tf::Quaternion::getIdentity(); }

  inline virtual const tf::Quaternion& process(const tf::Quaternion& x) {
    y0 = y0.slerp(x, a0);
    return y0;
  }
};

class OnePoleVector3 : public OnePole<tf::Vector3> {
public:
  OnePoleVector3() { zero(); }
  OnePoleVector3(double Fc, double Fs) : OnePole<tf::Vector3>(Fc, Fs) { zero(); }

  virtual void zero() { y0.setZero(); }

  inline virtual const tf::Vector3& process(const tf::Vector3& x) {
    this->y0 = a0*x + b1*this->y0;
    return this->y0;
  }
};

template <class T>
class BiQuad : public Filter<T>
{ // http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform/
public:
  BiQuad()
    : a0(0), a1(0), a2(0), b0(0), b1(0), b2(0) { }
  BiQuad(double Fc, double Fs)
    : a0(0), a1(0), a2(0), b0(0), b1(0), b2(0) { setFc(Fc, Fs); }

  inline void setFc(double Fc, double Fs) { setFc( Fc, Fs, 0.5); }

  inline void setFc(double Fc, double Fs, double Q) {
    double K = tan(M_PI*Fc/Fs);
    double k_quad_denom = K*K + K/Q + 1.0;
    a0 = K*K/k_quad_denom;
    a1 = 2*a0;
    a2 = a0;
    b0 = 1.0;
    b1 = 2*(K*K - 1.0)/k_quad_denom;
    b2 = (K*K - K/Q + 1.0)/k_quad_denom;
  }

  inline virtual const T& process(const T& x) {
    this->y0 = a0*x + a1*x1 + a2*x2 - b1*y1 - b2*y2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = this->y0;
    return this->y0;

//    w0 = x - b1*w1 - b2*w2;
//    this->y0 = a0*w0 + a1*w1 + a2*w2;
//    w2 = w1;
//    w1 = w0;
//    return this->y0;
  }

protected:
    double a0, a1, a2, b0, b1, b2;
    T x1, x2, y1, y2;
    //T w0, w1, w2;
};

class BiQuadVector3 : public BiQuad<tf::Vector3> {
public:
  BiQuadVector3() { zero(); }
  BiQuadVector3(double Fc, double Fs) : BiQuad<tf::Vector3>(Fc, Fs) { zero(); }

  virtual void zero() {
    //w0.setZero(); w1.setZero(); w2.setZero();
    this->y0.setZero(); y1.setZero(); y2.setZero(); x1.setZero(); x2.setZero();
  }
};

namespace razer_hydra
{

class RazerHydra
{
public:
  RazerHydra();
  ~RazerHydra();
  //usb_dev_handle *hid_dev;
  //int hid_dev_iface;
  //int hid_dev_ep_in;
  //uint16_t raw[NUM_SENSORS];
  int hidraw_fd;

  bool init(const char *device);
  bool poll(uint32_t ms_to_wait, float low_pass_corner_hz = 5.0);

  int16_t raw_pos[6], raw_quat[8];
  uint8_t raw_buttons[2];
  int16_t raw_analog[6];


  tf::Vector3 pos[2];
  tf::Quaternion quat[2];
  BiQuadVector3 filter_pos[2];
  OnePoleQuaternion filter_quat[2];
  float analog[6];
  uint8_t buttons[14];

  float period_estimate;
  ros::Time last_cycle_start;

};

}

#endif

