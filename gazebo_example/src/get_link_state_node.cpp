#include <ros/ros.h>
#include <gazebo_msgs/GetLinkState.h>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_link_state_node");
 
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    //ros::Publisher link_state_pub=n.advertise<gazebo_msgs::GetLinkState>("link_state", 1000);

    gazebo_msgs::GetLinkState link_state;
    ros::Rate loop_rate(10);

    link_state.request.link_name = "mbx/front_right_wheel";
    
    while(ros::ok()){
      client.call(link_state);
      ROS_INFO("position_x:%f", link_state.response.link_state.pose.position.x);
//    ROS_INFO("got link state");
      //link_state_pub.publish(link_state);
      ros::spinOnce();
      loop_rate.sleep();
      }
    return 0;
}
