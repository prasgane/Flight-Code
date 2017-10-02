#include "flight_code/flight_code.h"

namespace flight_code_ns
{
  flightCodeClass::flightCodeClass() :
    nh(ros::NodeHandle()),
    nh_private(ros::NodeHandle("~"))
  {
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, &flightCodeClass::state_cb,this);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, &flightCodeClass::pose_cb,this);
    RC_in_sub = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 1, &flightCodeClass::RCin_cb,this);

    pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10,this);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10,this);

    arming_client = nh_private.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh_private.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    nh_private.param<float>("takeoffAltitude",takeoff_height,1); //default 1m
    nh_private.param<float>("forwardVel",x_vel,0.5); //default 0.5m/s
    nh_private.param<float>("vel_time",x_vel_duration,5); //default 5 sec
    nh_private.param<float>("safety_lim",safety_limit,1); //default 5 sec
    stop_vel.twist.linear.x = -0.5;
    vel_pose_x.twist.linear.x = x_vel;

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    disarm_cmd.request.value = false;
    last_request = ros::Time::now();

    setpoint_type = 0;
    setpoint_pose = 0;
    setpoint_att = 1;
    setpoint_vel = 2;

    armed = false;
    takeoff = false;
    man1 = false;
    man2 = false;
    man3 = false;
    man4 = false;
    man5 = false;
    man6 = false;
    landed = false;
    arm_switch = false;
    offb = false;
    flight_ready = false;
    sitl = false;
    Dswitch = false;
    Dswitch_out = true;
    safety_switch = false;

  }

  void flightCodeClass::state_cb(const mavros_msgs::State::ConstPtr& msg)
  {
      current_state = *msg;
  }

  void flightCodeClass::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      current_pose = *msg;
  }

  void flightCodeClass::RCin_cb(const mavros_msgs::RCIn::ConstPtr& msg)
  {
      current_RCin = *msg;
      for(int i = 0; i < 12; i++)
      {
          Arr[i] = msg->channels[i];
      }
  }

  void flightCodeClass::pose_calculator()
  {
    pose = current_pose;
    takeoff_pose = current_pose;
    takeoff_pose.pose.position.z = takeoff_height; // will take off to specified height
    //maintaining the initial oreientation and position.
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        pos_pub.publish(current_pose);
        ros::spinOnce();
    }

    while(ros::ok())
    {
        // Start of SITL switch
        // If sitl = true, arm and set mode to offboard
        // If sitl = false, arming and mode switches are done by TX
        if(sitl)
        {
          if( current_state.mode != "OFFBOARD" &&
              (ros::Time::now() - last_request > ros::Duration(5.0)))
              {
                if( set_mode_client.call(offb_set_mode) )
                {
                    ROS_INFO("Offboard enabled");
                    offb = true;
                }
                last_request = ros::Time::now();
            }
            else
            {
                if( !current_state.armed &&
                   (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if( arming_client.call(arm_cmd) &&
                       arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                        armed = true;
                    }
                    last_request = ros::Time::now();
                }
            }
        // Else: wait for TX commands
        }
        else
        {
            // Wait for system to arm
          if(!current_state.armed)
          {
              if(ros::Time::now() - last_request > ros::Duration(1.0))
              {
                  ROS_INFO("System not armed, waiting for arm...");
                  armed = false;
                  arm_switch = false;
                  last_request = ros::Time::now();
              }
          }
          else
          {
             if(!arm_switch)
             {
                 armed = true;
                 arm_switch = true;
                 ROS_INFO("System has been armed, waiting for offboard mode...");
             }
          }
            // Wait for Offboard mode to start flight test
            if(!offb && armed && current_state.mode == "OFFBOARD")
            {
                offb = true;
                ROS_INFO("Flight Mode changed to OFFBOARD, starting offboard control");
                ROS_INFO("Sending Pose: home_pose");
                last_request = ros::Time::now();
            }
            else
            {
                if(!offb && (ros::Time::now() - last_request > ros::Duration(2.0)))
                {
                    ROS_INFO("Waiting for offboard mode...");
                    last_request = ros::Time::now();
                }
            }
        } // End of SITL switch

        // Are we ready to take off?
        if(offb && armed && !flight_ready)
        {
            flight_ready = true;
            ROS_INFO("Armed and in offboard control mode");
        }

        // Takeoff
        if(flight_ready && !man1)
        {
            if(ros::Time::now() - last_request < ros::Duration(1.0)){
                pose.header.stamp = ros::Time::now();
        }
        else
        {
                ROS_INFO("Moving to takeoff");
                pose.header.stamp = ros::Time::now();
                pose = takeoff_pose;
                man1 = true;
                last_request = ros::Time::now();
            }
        }

        // Move to takeoff
        if(man1 && !takeoff)
        {
            if(ros::Time::now() - last_request < ros::Duration(3.0))
            {
                pose.header.stamp = ros::Time::now();
            }
            else
            {
                ROS_INFO("Moving to maneuver start pose");
                pose.header.stamp = ros::Time::now();
                takeoff = true;
                last_request = ros::Time::now();
            }
        }

        // Check D-Switch Value for internal mode switching
        if(Arr[5] < 1500)
        {
            Dswitch = false;
        }
        else if ((Arr[5] >= 1500) && (Dswitch_out))
        {
            Dswitch = true;
            // This handles the time difference between
            // takeoff and entering forward motion
            last_request = ros::Time::now();
            Dswitch_out = false;
        }
        // Provide system feedback during waiting period
        if(takeoff && !man2 && !Dswitch)
        {
            ROS_INFO_THROTTLE(2, "Waiting for D-Switch flip to begin forward motion");
        }

        // Wait for D switch flip to begin commanding velocities
        if(takeoff && !man2 && Dswitch)
        {
            if(ros::Time::now() - last_request < ros::Duration(.5))
            {
                pose.header.stamp = ros::Time::now();
            }
            else
            {
                ROS_INFO("Starting Velocity Maneuver: Stop Velocity");
                setpoint_type = setpoint_vel;
                velocity = stop_vel;
                velocity.header.stamp = ros::Time::now();
                man2 = true;
                last_request = ros::Time::now();
            }
        }

        // Stop for 1 second, then start forward motion
        // Start incorporating yaw commands to to the velocity message
        if(man2 && !man3)
        {
            if(ros::Time::now() - last_request < ros::Duration(.5))
            {
                velocity.header.stamp = ros::Time::now();
            }
            else
            {
                ROS_INFO("Velocity Maneuver: Positive Velocity .5 m/s");
                velocity = vel_pose_x;
                velocity.header.stamp = ros::Time::now();
                man3 = true;
                last_request = ros::Time::now();
            }
        }

        // Move forward for 10 seconds
        if(man3 && !man4)
        {
            // Continue forward motion until safety limits are reached
            if(current_pose.pose.position.x > safety_limit)
            {
                safety_switch = true;
            }

            if(!safety_switch)
            {
                velocity.header.stamp = ros::Time::now();
            }
            else
            {
                ROS_INFO("Velocity Maneuver: Stop Velocity");
                velocity = stop_vel;
                velocity.header.stamp = ros::Time::now();
                man4 = true;
                last_request = ros::Time::now();
            }
        }

        // Stop the velocity, then start holding  current pose
        if(man4 && !man5)
        {
            if(ros::Time::now() - last_request < ros::Duration(1.0))
            {
                velocity.header.stamp = ros::Time::now();
            }
            else
            {
                ROS_INFO("Holding current position");
                setpoint_type = setpoint_pose;
                pose = current_pose;
                // pose.pose.orientation.x = 0.0;
                // pose.pose.orientation.y = 0.0;
                // pose.pose.orientation.z = 0.0;
                // pose.pose.orientation.w = -1.0;
                pose.header.stamp = ros::Time::now();
                man5 = true;
                last_request = ros::Time::now();
            }
        }

        // Hold current pose then land
        if(man5 && !man6){
            if(ros::Time::now() - last_request < ros::Duration(2.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Landing...");
                pose.pose.position.z = -0.15;
                pose.header.stamp = ros::Time::now();
                man6 = true;
                last_request = ros::Time::now();
            }
        }


        // Land
        if(man6 && !landed){
            if(ros::Time::now() - last_request < ros::Duration(3.0)){
                pose.header.stamp = ros::Time::now();
            } else {
                ROS_INFO("Landing...");
                landed = true;
                last_request = ros::Time::now();
            }
        }


        // Publish setpoints and record status for MATLAB
        if(setpoint_type == setpoint_pose){
            pos_pub.publish(pose);
        } else if(setpoint_type == setpoint_att){
           // local_att_pub.publish(attitude);
        } else if(setpoint_type == setpoint_vel){
            vel_pub.publish(velocity);
        }

        // Disarm the system if not done manually upon landing;
        if(landed && current_state.armed)
        {
            if(arming_client.call(disarm_cmd) && disarm_cmd.response.success)
            {
                ROS_INFO("Vehicle disarmed");
            }
        }
        else if(landed && armed && !current_state.armed)
        {
            ROS_INFO("System disarmed manually, exiting program...");
        }

        ros::spinOnce();

    }

  }


}
