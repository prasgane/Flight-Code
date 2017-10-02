#ifndef FLIGHT_CODE_H
#define FLIGHT_CODE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <math.h>

namespace flight_code_ns
{
  class flightCodeClass
  {

  public:
    flightCodeClass();
    void pose_calculator();

  private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber RC_in_sub;

    ros::Publisher pos_pub;
    ros::Publisher vel_pub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    geometry_msgs::PoseStamped home_pose;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped takeoff_pose;
    geometry_msgs::PoseStamped landing_pose;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::TwistStamped velocity;
    geometry_msgs::TwistStamped vel_pose_x;
    geometry_msgs::TwistStamped stop_vel;
    mavros_msgs::RCIn current_RCin;
    mavros_msgs::State current_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandBool disarm_cmd;
    ros::Time last_request;

    bool armed;
    bool takeoff;
    bool man1;
    bool man2;
    bool man3;
    bool man4;
    bool man5;
    bool man6;
    bool landed;
    bool arm_switch;
    bool offb;
    bool flight_ready;
    bool sitl;
    bool Dswitch;
    bool Dswitch_out;
    bool safety_switch;
    int Arr[12];
    int setpoint_type;
    int setpoint_pose;
    int setpoint_att;
    int setpoint_vel;
    float takeoff_height, x_vel, x_vel_duration,safety_limit;

    void state_cb();
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void RCin_cb(const mavros_msgs::RCIn::ConstPtr& msg);

  };

}

#endif
