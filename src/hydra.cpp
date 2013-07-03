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

#include "razer_hydra/hydra.h"
#include <tf/tf.h>  // could remove TF dependency if we use geometry_msgs
#include "ros/console.h"
#include "ros/assert.h"

#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <cstring>

// loosely adapted from the following 
// https://github.com/rpavlik/vrpn/blob/razer-hydra/vrpn_Tracker_RazerHydra.C

// and with reference to
// http://lxr.free-electrons.com/source/samples/hidraw/hid-example.c

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 *
 * If you need this, please have your distro update the kernel headers.
 */
#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

// eventually crawl hidraw file system using this:
// http://www.signal11.us/oss/udev/


namespace razer_hydra {

RazerHydra::RazerHydra()
  : hidraw_fd(0)
{
  ros::Time::init();
  last_cycle_start = ros::Time::now();
  period_estimate.setFc(0.11, 1.0); // magic number for 50% mix at each step
  period_estimate.setValue(0.004);

  for (int i = 0; i < 2; i++)
  {
    pos[i] = tf::Vector3(0,0,0);
    quat[i] = tf::Quaternion(0,0,0,1);
  }
}

RazerHydra::~RazerHydra()
{
  if (hidraw_fd >= 0)
  {
    ROS_INFO("releasing hydra");
    uint8_t buf[256];
    memset(buf, 0, sizeof(buf));
    buf[6] = 1;
    buf[8] = 4;
    buf[89] = 5;
    int res = ioctl(hidraw_fd, HIDIOCSFEATURE(91), buf);
    if (res < 0)
    {
      ROS_ERROR("unable to stop streaming");
      perror("HIDIOCSFEATURE");
    }
    else
      ROS_INFO("stopped streaming");
    close(hidraw_fd);
  }
}

bool RazerHydra::init(const char *device)
{
    int res;
    uint8_t buf[256];
    struct hidraw_report_descriptor rpt_desc;
    struct hidraw_devinfo info;

    hidraw_fd = open(device, O_RDWR | O_NONBLOCK);
    if (hidraw_fd < 0)
    {
        ROS_ERROR("couldn't open hidraw device");
        return false;
    }
    ROS_DEBUG("%s", (std::string("opened device") + std::string(device)).c_str());

    memset(&rpt_desc, 0x0, sizeof(rpt_desc));
    memset(&info,     0x0, sizeof(info));
    memset(buf,       0x0, sizeof(buf));

    /*
    // Get Report Descriptor Size
    ROS_DEBUG("Getting Report Descriptor Size");
    res = ioctl(hidraw_fd, HIDIOCGRDESCSIZE, &desc_size);
    if (res < 0)
        perror("HIDIOCGRDESCSIZE");
    else
        printf("Report Descriptor Size: %d\n", desc_size);

    // Get Report Descriptor
    ROS_DEBUG("Getting Report Descriptor");
    rpt_desc.size = desc_size;
    res = ioctl(hidraw_fd, HIDIOCGRDESC, &rpt_desc);
    if (res < 0) {
        perror("HIDIOCGRDESC");
    } else {
        printf("Report Descriptor:\n");
        for (size_t i = 0; i < rpt_desc.size; i++)
        printf("%hhx ", rpt_desc.value[i]);
        puts("\n");
    }
    */

    /* Get Raw Name */
    ROS_DEBUG("Getting Raw Name");
    res = ioctl(hidraw_fd, HIDIOCGRAWNAME(256), buf);
    if (res < 0)
        perror("HIDIOCGRAWNAME");
    else
        ROS_DEBUG("Raw Name: %s\n", buf);

    // set feature to start it streaming
    memset(buf, 0x0, sizeof(buf));
    buf[6] = 1;
    buf[8] = 4;
    buf[9] = 3;
    buf[89] = 6;
    int attempt = 0;
    for (; attempt < 50; attempt++)
    {
        res = ioctl(hidraw_fd, HIDIOCSFEATURE(91), buf);
        if (res < 0)
        {
            ROS_ERROR("unable to start streaming");
            perror("HIDIOCSFEATURE");
            usleep(50000);
        }
        else
        {
            ROS_INFO("Device stream is working.");
            break;
        }
    }
    ROS_DEBUG("%d attempts", attempt);
    return attempt < 60;
}

bool RazerHydra::poll(uint32_t ms_to_wait, float low_pass_corner_hz)
{
  if (hidraw_fd < 0)
  {
    ROS_ERROR("hidraw device is not open, couldn't poll.");
    return false;
  }
  if(ms_to_wait == 0)
  {
    ROS_ERROR("ms_to_wait must be at least 1.");
    return false;
  }
  if(low_pass_corner_hz <= std::numeric_limits<float>::epsilon())
  {
    ROS_ERROR("Corner frequency for low-pass filter must be greater than 0. Aborting.");
    return false;
  }
  ros::Time t_start(ros::Time::now());
  ros::Time t_deadline(t_start + ros::Duration(0.001 * ms_to_wait));

  uint8_t buf[64];
  while (ros::Time::now() < t_deadline)
  {
    ssize_t nread = read(hidraw_fd, buf, sizeof(buf));
    //ROS_INFO("read %d bytes", (int)nread);
    if (nread > 0)
    {
      static bool first_time = true;
      // Update average read period
      if(!first_time) {
        float last_period = (ros::Time::now() - last_cycle_start).toSec();
        period_estimate.process(last_period);
        //ROS_INFO("last_cycle: %.4f sec, average: %.4f sec", last_period, period_estimate);
      }
      last_cycle_start = ros::Time::now();
      if(first_time)
      {
        first_time = false;
        ROS_INFO("Got first data, everything should be working now!");
      }

      // Update filter frequencies
      float Fs = 1/period_estimate.getValue();
      float Fc = low_pass_corner_hz;
      for (int i = 0; i < 2; i++)
      {
        filter_pos[i].setFc(Fc, Fs);
        filter_quat[i].setFc(Fc, Fs);
      }

      // Read data
      raw_pos[0] = *((int16_t *)(buf+8));
      raw_pos[1] = *((int16_t *)(buf+10));
      raw_pos[2] = *((int16_t *)(buf+12));
      raw_quat[0] = *((int16_t *)(buf+14));
      raw_quat[1] = *((int16_t *)(buf+16));
      raw_quat[2] = *((int16_t *)(buf+18));
      raw_quat[3] = *((int16_t *)(buf+20));
      raw_buttons[0] = buf[22] & 0x7f;
      raw_analog[0] = *((int16_t *)(buf+23));
      raw_analog[1] = *((int16_t *)(buf+25));
      raw_analog[2] = buf[27];

      raw_pos[3] = *((int16_t *)(buf+30));
      raw_pos[4] = *((int16_t *)(buf+32));
      raw_pos[5] = *((int16_t *)(buf+34));
      raw_quat[4] = *((int16_t *)(buf+36));
      raw_quat[5] = *((int16_t *)(buf+38));
      raw_quat[6] = *((int16_t *)(buf+40));
      raw_quat[7] = *((int16_t *)(buf+42));
      raw_buttons[1] = buf[44] & 0x7f;
      raw_analog[3] = *((int16_t *)(buf+45));
      raw_analog[4] = *((int16_t *)(buf+47));
      raw_analog[5] = buf[49];

      // mangle the reported pose into the ROS frame conventions
      tf::Matrix3x3 ros_to_razer(  0, -1,  0,
                                  -1,  0,  0,
                                   0,  0, -1 );
      for (int i = 0; i < 2; i++)
      {
        pos[i].setX(raw_pos[3*i+0] * 0.001);
        pos[i].setY(raw_pos[3*i+1] * 0.001);
        pos[i].setZ(raw_pos[3*i+2] * 0.001);
        pos[i] = ros_to_razer*pos[i];
        
        tf::Quaternion q(raw_quat[i*4+1] / 32768.0,
                         raw_quat[i*4+2] / 32768.0,
                         raw_quat[i*4+3] / 32768.0,
                         raw_quat[i*4+0] / 32768.0);
        
        tf::Matrix3x3 mat(q);
        mat = ros_to_razer*mat*tf::Matrix3x3(tf::createQuaternionFromRPY(0, 0, M_PI_2));
        mat.getRotation(quat[i]);
      }

      // Apply filters
      for (int i = 0; i < 2; i++)
      {
        quat[i] = filter_quat[i].process(quat[i]);
        pos[i] = filter_pos[i].process(pos[i]);
      }

      analog[0] = raw_analog[0] / 32768.0;
      analog[1] = raw_analog[1] / 32768.0;
      analog[2] = raw_analog[2] / 255.0;
      analog[3] = raw_analog[3] / 32768.0;
      analog[4] = raw_analog[4] / 32768.0;
      analog[5] = raw_analog[5] / 255.0;

      for (int i = 0; i < 2; i++)
      {
        buttons[i*7  ] = (raw_buttons[i] & 0x01) ? 1 : 0;
        buttons[i*7+1] = (raw_buttons[i] & 0x04) ? 1 : 0;
        buttons[i*7+2] = (raw_buttons[i] & 0x08) ? 1 : 0;
        buttons[i*7+3] = (raw_buttons[i] & 0x02) ? 1 : 0;
        buttons[i*7+4] = (raw_buttons[i] & 0x10) ? 1 : 0;
        buttons[i*7+5] = (raw_buttons[i] & 0x20) ? 1 : 0;
        buttons[i*7+6] = (raw_buttons[i] & 0x40) ? 1 : 0;
      }

      return true;
    }
    else
    {
      ros::Time to_sleep = last_cycle_start + ros::Duration(period_estimate.getValue()*0.95);
      float sleep_duration = (to_sleep - ros::Time::now()).toSec();
      if(sleep_duration > 0)
      {
        //ROS_INFO("Data not ready, sleeping for %.6f sec", sleep_duration);
        ros::Time::sleepUntil(to_sleep);
      }
      else {
        //ROS_INFO("Data not ready, doing default sleep of 500 us");
        usleep(250);
      }

    }
  }
  //ROS_INFO("Ran out of time, returning!");
  return false;
}

} //namespace
