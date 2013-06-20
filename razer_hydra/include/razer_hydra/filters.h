#ifndef RAZER_HYDRA_FILTERS_H
#define RAZER_HYDRA_FILTERS_H

#include <tf/tf.h>
#include <ros/ros.h>

template <class T>
class Filter {
public:
  virtual void setValue(const T& val) { y0 = val; }
  virtual void setFc(double Fc, double Fs) = 0;
  inline virtual const T& getValue() { return y0; }
  //inline const T& process(const T& x) = 0;

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

  inline const T& process(const T& x) {
    this->y0 = a0*x + b1*this->y0;
    return this->y0;
  }

protected:
    double a0, b1;
};

class OnePoleQuaternion : public OnePole<tf::Quaternion> {
public:
  OnePoleQuaternion() { setValue(tf::Quaternion(0,0,0,1)); }
  OnePoleQuaternion(double Fc, double Fs) : OnePole<tf::Quaternion>(Fc, Fs) { setValue(tf::Quaternion(0,0,0,1)); }

  inline const tf::Quaternion& process(const tf::Quaternion& x) {
    y0 = y0.slerp(x, a0);
    return y0;
  }
};

class OnePoleVector3 : public OnePole<tf::Vector3> {
public:
  OnePoleVector3() { setValue(tf::Vector3(0,0,0)); }
  OnePoleVector3(double Fc, double Fs) : OnePole<tf::Vector3>(Fc, Fs) { setValue(tf::Vector3(0,0,0)); }

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

  virtual void setValue(const T& val) { this->y0 = y1 = y2 = x1 = x2 = val; }

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
  BiQuadVector3() { setValue(tf::Vector3(0,0,0)); }
  BiQuadVector3(double Fc, double Fs) : BiQuad<tf::Vector3>(Fc, Fs) { setValue(tf::Vector3(0,0,0)); }
};


#endif // RAZER_HYDRA_FILTERS_H
