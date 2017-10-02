#include <ros/ros.h>
#include <flight_code/flight_code.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"flight_node");
  ros::NodeHandle nh;

  flight_code_ns::flightCodeClass hello;
  hello.pose_calculator();
  ros::spin();

  return 0;
}
