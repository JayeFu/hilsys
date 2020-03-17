#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "come_close_node");
 
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Publisher cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    gazebo_msgs::GetModelState mbx_state;
    gazebo_msgs::GetModelState fwx_state;
    geometry_msgs::Twist vel;
    ros::Rate loop_rate(10);

    mbx_state.request.model_name = "mbx";
    fwx_state.request.model_name = "fwx";
   
    while(ros::ok()){
      client.call(mbx_state);
      client.call(fwx_state);
      ROS_INFO("fwx_position_x:%f", fwx_state.response.pose.position.x);
      ROS_INFO("mbx_position_x:%f", mbx_state.response.pose.position.x);
      if((mbx_state.response.pose.position.x) - (fwx_state.response.pose.position.x) > 2.3){
        vel.linear.x = 1;
      }
      else{
        vel.linear.x = 0;
      }
      cmd_vel_pub.publish(vel);
//    ROS_INFO("got model state");
      //model_state_pub.publish(model_state);
      ros::spinOnce();
      loop_rate.sleep();
      }
    return 0;
}
