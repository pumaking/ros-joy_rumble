#include <ros/ros.h>
#include "rumble.h"


int
main(int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "joy_rumble");
  ros::NodeHandle n;
  Rumble rumble;
  rumble.init(n);
//  rumble.write_force(0x0ff0, 0x0, 0x0, 0xffff);
//  sleep(1);
//
//  rumble.write_force(0x0ff0, 0x4000, 0x0, 0xffff);
//  sleep(1);
//
//  rumble.write_force(0x0ff0, 0x8000, 0x0, 0xffff);
//  sleep(5);
//
//  rumble.write_force(0x0ff0, 0xb000, 0x0, 0xffff);
//  sleep(5);
//
//  rumble.write_force(0x0ff0, 0xf000, 0x0, 0xffff);
  ros::spin();
}
