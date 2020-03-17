#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_model_state_node");
 
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    //ros::Publisher model_state_pub=n.advertise<gazebo_msgs::GetModelState>("model_state", 1000);

    gazebo_msgs::GetModelState model_state;
    ros::Rate loop_rate(10);

    model_state.request.model_name = "mbx";
   
    while(ros::ok()){
      client.call(model_state);
      ROS_INFO("position_x:%f", model_state.response.pose.position.x);
//    ROS_INFO("got model state");
      //model_state_pub.publish(model_state);
      ros::spinOnce();
      loop_rate.sleep();
      }
    return 0;
}
