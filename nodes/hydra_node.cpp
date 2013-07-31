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

#include "sensor_msgs/Joy.h"

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

    int polling_ms = 5;
    n_private.param<int>("polling_ms", polling_ms, 5);
    if( polling_ms <= 0 ) {
      ROS_WARN("polling_ms must be >= 1. Setting to default rate of 5 ms.");
      polling_ms = 5;
    }
//    ros::Rate rate(1/(0.001*polling_ms));


    double corner_hz = 2.5;
    n_private.param<double>("corner_hz", corner_hz, 2.5);
    ROS_INFO("Using filter corner frequency of %.1f Hz", corner_hz);


    double pivot_arr[3];
    n_private.param<double>("pivot_x", pivot_arr[0], 0.0);
    n_private.param<double>("pivot_y", pivot_arr[1], 0.0);
    n_private.param<double>("pivot_z", pivot_arr[2], 0.0);
    tf::Vector3 pivot_vec(pivot_arr[0], pivot_arr[1], pivot_arr[2]);

    double grab_arr[3];
    n_private.param<double>("grab_x", grab_arr[0], 0.0);
    n_private.param<double>("grab_y", grab_arr[1], 0.0);
    n_private.param<double>("grab_z", grab_arr[2], 0.0);
    tf::Vector3 grab_vec(grab_arr[0], grab_arr[1], grab_arr[2]);

    bool use_grab_frame = false;
    n_private.param<bool>("use_grab_frame", use_grab_frame, false);

    ROS_INFO("Setting pivot offset [%.3f, %.3f, %.3f]", pivot_vec.x(), pivot_vec.y(), pivot_vec.z());
    ROS_INFO("Setting grab offset [%.3f, %.3f, %.3f]", grab_vec.x(), grab_vec.y(), grab_vec.z());
    if(use_grab_frame)
      ROS_INFO("Sending messages using the 'grab' frame.");
    else
      ROS_INFO("Sending messages using the 'pivot' frame.");

    if(publish_tf)
      ROS_INFO("Publishing frame data to TF.");

    // Initialize ROS stuff
    ros::Publisher raw_pub = n.advertise<razer_hydra::HydraRaw>("hydra_raw", 1);
    ros::Publisher calib_pub = n.advertise<razer_hydra::Hydra>("hydra_calib", 1);
    ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>("hydra_joy", 1);
    tf::TransformBroadcaster *broadcaster = 0;
    if(publish_tf) broadcaster = new tf::TransformBroadcaster();


    RazerHydra hydra;
    ROS_INFO("opening hydra on %s", device.c_str());
    if (!hydra.init(device.c_str()))
    {
      ROS_FATAL("couldn't open hydra on %s", device.c_str());
      return 1;
    }
    ROS_INFO("starting stream... (first packet takes a few seconds)");
    while (n.ok())
    {
      if (hydra.poll(polling_ms, corner_hz))
      {
        // publish raw data
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

        // publish calibrated data
        razer_hydra::Hydra h_msg;
        h_msg.header.stamp = msg.header.stamp;
        for (int i = 0; i < 2; i++)
        {
          tf::Transform transform(hydra.quat[i], hydra.pos[i]); // original transform!
          tf::Transform t_pivot = transform, t_grab = transform;

          // pivot
          t_pivot.setOrigin(transform.getOrigin() + transform.getBasis()*pivot_vec);
          tf::transformTFToMsg(t_pivot, transforms[i].transform);

          // grab point
          t_grab.setOrigin(transform.getOrigin() + transform.getBasis()*grab_vec);
          tf::transformTFToMsg(t_grab, transforms[2+i].transform);

          h_msg.paddles[i].transform = use_grab_frame ? transforms[2+i].transform : transforms[i].transform;
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
          std::string frames[4] = {"hydra_left_pivot", "hydra_right_pivot", "hydra_left_grab", "hydra_right_grab"};
          for(int kk = 0; kk < 4; kk++)
          {
            transforms[kk].header.stamp = h_msg.header.stamp;
            transforms[kk].header.frame_id = "hydra_base";
            transforms[kk].child_frame_id = frames[kk];
          }

          broadcaster->sendTransform(transforms);
        }

        // emulate PS3 joystick message
        sensor_msgs::Joy joy_msg;

        joy_msg.axes.resize(16);
        joy_msg.buttons.resize(16);
        joy_msg.header.stamp = h_msg.header.stamp;

        // analog pads
        // left
        joy_msg.axes[0] = -h_msg.paddles[0].joy[0]; // x
        joy_msg.axes[1] = h_msg.paddles[0].joy[1]; // y
        joy_msg.buttons[1] = h_msg.paddles[0].buttons[6]; // push
        //right
        joy_msg.axes[2] = -h_msg.paddles[1].joy[0]; // x
        joy_msg.axes[3] = h_msg.paddles[1].joy[1]; // y
        joy_msg.buttons[2] = h_msg.paddles[1].buttons[6]; // push

        // push buttons
        joy_msg.buttons[7] = h_msg.paddles[0].buttons[1]; // left
        joy_msg.buttons[5] = h_msg.paddles[0].buttons[3]; // right
        joy_msg.buttons[6] = h_msg.paddles[0].buttons[2]; // down
        joy_msg.buttons[4] = h_msg.paddles[0].buttons[4]; // up

        joy_msg.buttons[15] = h_msg.paddles[1].buttons[1]; // square
        joy_msg.buttons[13] = h_msg.paddles[1].buttons[3]; // circle
        joy_msg.buttons[14] = h_msg.paddles[1].buttons[2]; // triangle
        joy_msg.buttons[12] = h_msg.paddles[1].buttons[4]; // cross

        // menu buttons
        joy_msg.buttons[0] = h_msg.paddles[0].buttons[5]; // select
        joy_msg.buttons[3] = h_msg.paddles[1].buttons[5]; // start

        // top buttons
        joy_msg.buttons[10] = h_msg.paddles[0].buttons[0]; // L1
        joy_msg.buttons[8] = h_msg.paddles[0].trigger > 0.15; // L2
        joy_msg.axes[8] = h_msg.paddles[0].trigger; // L2

        joy_msg.buttons[11] = h_msg.paddles[1].buttons[0]; // R1
        joy_msg.buttons[9] = h_msg.paddles[1].trigger > 0.15; // R2
        joy_msg.axes[9] = h_msg.paddles[1].trigger; // R2

        // fill in fake axes for binary buttons
        for ( int i=4; i<8; i++ )
        {
          joy_msg.axes[i] = joy_msg.buttons[i] ? 1.0 : 0.0;
        }
        for ( int i=10; i<16; i++ )
        {
          joy_msg.axes[i] = joy_msg.buttons[i] ? 1.0 : 0.0;
        }

        joy_pub.publish(joy_msg);

        ros::spinOnce();
      }
    }

    // clean up
    if(broadcaster) delete broadcaster;
    return 0;
}

