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

#include <razer_hydra/filters.h>
#include <tf/tf.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/hidraw.h>
#include <libusb-1.0/libusb.h>

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
  OnePoleVector3 filter_pos[2];
  OnePoleQuaternion filter_quat[2];
  float analog[6];
  uint8_t buttons[14];

  OnePole<float> period_estimate;
  ros::Time last_cycle_start;

};

}

#endif

