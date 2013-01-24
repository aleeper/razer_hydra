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
#include <string>
#include <ros/ros.h>
#include "razer_hydra/hydra.h"
#include "razer_hydra/HydraRaw.h"
#include "razer_hydra/Hydra.h"
#include "tf/tf.h"

// Visualization
#include <tf/transform_broadcaster.h>

using namespace razer_hydra;
using std::string;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hydra_driver");
    ros::NodeHandle n, n_private("~");

    // Get configuration params
    string device;
    n_private.param<std::string>("device", device, "/dev/hidraw0");
    bool publish_tf = false;
    n_private.param<bool>("publish_tf", publish_tf, false);
    int polling_ms = 10;
    n_private.param<int>("polling_ms", polling_ms, 10);
    double lambda_filter = 0.5;
    n_private.param<double>("lambda_filter", lambda_filter, 0.5);
    double offset_linear[3];
    n_private.param<double>("offset_x", offset_linear[0], 0.0);
    n_private.param<double>("offset_y", offset_linear[1], 0.0);
    n_private.param<double>("offset_z", offset_linear[2], 0.0);
    tf::Vector3 offset(offset_linear[0], offset_linear[1], offset_linear[2]);

    ROS_INFO("Setting linear offset [%.3f, %.3f, %.3f]", offset.x(), offset.y(), offset.z());

    // Initialize ROS stuff
    ros::Publisher raw_pub = n.advertise<razer_hydra::HydraRaw>("hydra_raw", 1);
    ros::Publisher calib_pub = n.advertise<razer_hydra::Hydra>("hydra_calib", 1);
    tf::TransformBroadcaster *broadcaster = 0;
    if(publish_tf) broadcaster = new tf::TransformBroadcaster();


    RazerHydra hydra;
    ROS_INFO("opening hydra on %s", device.c_str());
    if (!hydra.init(device.c_str()))
    {
      ROS_FATAL("couldn't open hydra on %s", device.c_str());
      return 1;
    }
    ROS_INFO("starting stream...");
    while (n.ok())
    {
      if (hydra.poll(polling_ms, lambda_filter))
      {
        razer_hydra::HydraRaw msg;
        msg.header.stamp = ros::Time::now();
        for (int i = 0; i < 6; i++)
          msg.pos[i] = hydra.raw_pos[i];
        for (int i = 0; i < 8; i++)
          msg.quat[i] = hydra.raw_quat[i];
        for (int i = 0; i < 2; i++)
          msg.buttons[i] = hydra.raw_buttons[i];
        for (int i = 0; i < 6; i++)
          msg.analog[i] = hydra.raw_analog[i];
        raw_pub.publish(msg);

        std::vector<geometry_msgs::TransformStamped> transforms(4);

        razer_hydra::Hydra h_msg;
        h_msg.header.stamp = msg.header.stamp;
        for (int i = 0; i < 2; i++)
        {
          tf::Transform transform(hydra.quat[i], hydra.pos[i]);
          tf::transformTFToMsg(transform, transforms[i].transform);
          transform.setOrigin(transform.getOrigin() + transform.getBasis()*offset);
          tf::transformTFToMsg(transform, transforms[2+i].transform);
          h_msg.paddles[i].transform = transforms[i].transform;
        }
        for (int i = 0; i < 7; i++)
        {
          h_msg.paddles[0].buttons[i] = hydra.buttons[i];
          h_msg.paddles[1].buttons[i] = hydra.buttons[i+7];
        }
        for (int i = 0; i < 2; i++)
        {
          h_msg.paddles[0].joy[i] = hydra.analog[i];
          h_msg.paddles[1].joy[i] = hydra.analog[i+3];
        }
        h_msg.paddles[0].trigger = hydra.analog[2];
        h_msg.paddles[1].trigger = hydra.analog[5];
        calib_pub.publish(h_msg);

        if(broadcaster)
        {
          std::string frames[4] = {"hydra_left", "hydra_right", "hydra_left_offset", "hydra_right_offset"};
          for(int kk = 0; kk < 4; kk++)
          {
            transforms[kk].header.stamp = h_msg.header.stamp;
            transforms[kk].header.frame_id = "hydra_base";
            transforms[kk].child_frame_id = frames[kk];
          }

          broadcaster->sendTransform(transforms);
        }

        ros::spinOnce();
      }
    }

    // clean up
    if(broadcaster) delete broadcaster;
    return 0;
}

