#define temp 2.0

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "wx_rotate_node");
  ros::NodeHandle nh;
  
  ros::Publisher rotate_msg_pub = nh.advertise<std_msgs::Float64>("command", 1000);
  ros::Rate loop_rate(100);
  double init_time = ros::Time::now().toSec();

  while(ros::ok()){
    std_msgs::Float64 rotate_msg;
    //temp = ros::Time::now().toSec();
    rotate_msg.data = (ros::Time::now().toSec() - init_time) * temp;
    ROS_INFO("time: %f",rotate_msg.data);
    rotate_msg_pub.publish(rotate_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
