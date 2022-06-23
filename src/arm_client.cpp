#include "ros/ros.h"
#include "arm_srv/arm.h"
#include <cstdlib>
#include <std_msgs/Int32.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_client");
  if (argc != 2)
  {
    ROS_INFO("usage: arm_client <int>");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<arm_srv::arm>("arm");
  ros::Publisher down_pub = n.advertise<std_msgs::Int32>("arm_down", 1000); //1
  ros::Publisher up_pub = n.advertise<std_msgs::Int32>("arm_up", 1000); //2
  ros::Publisher brake_pub = n.advertise<std_msgs::Int32>("arm_brake", 1000); //0

  arm_srv::arm srv;

  srv.request.arm_request = atoll(argv[1]);
  
  std_msgs::Int32 msg;
  msg.data = srv.request.arm_request;

  if (client.call(srv))
  {

    if(srv.request.arm_request == 0){
        sleep(1.5); //delay to setup publisher
        brake_pub.publish(msg);
        ROS_INFO("Client request: Brake");
    }
    else if (srv.request.arm_request == 1 ){
        sleep(1.5); //delay to setup publisher
        down_pub.publish(msg);
        ROS_INFO("Client request: Move Down");
    }
    else if (srv.request.arm_request == 2 ){
        sleep(1.5); //delay to setup publisher
        up_pub.publish(msg);
        ROS_INFO("Client request: Move Up");
    }
    else if (srv.request.arm_request == 3 ){
        ROS_INFO("Client request: Check Status");
        // ROS_INFO("Arm_position: %d", srv.response.arm_position);
        // ROS_INFO("Arm_moving_state: %d", srv.response.arm_moving_state);
        // Print arm position
        if (srv.response.arm_position == 1){
            ROS_INFO("Arm position: In between");
        }
        else if (srv.response.arm_position == 0){
            ROS_INFO("Arm position: Bottom");
        }
        else if (srv.response.arm_position == 2){
            ROS_INFO("Arm position: Top");
        }
        else if (srv.response.arm_position == -1){
            ROS_INFO("Arm position: Unknown");
        }
        else{
            ROS_INFO("arm_srv client response callback error");
        }
        // Print arm moving state
        if (srv.response.arm_moving_state == 1){
            ROS_INFO("Moving State: Going Down");
        }
        else if (srv.response.arm_moving_state == 0){
            ROS_INFO("Moving State: Brake");
        }
        else if (srv.response.arm_moving_state == 2){
            ROS_INFO("Moving State: Going Up");
        }
        else if (srv.response.arm_moving_state == -1){
            ROS_INFO("Arm position: Unknown");
        }
        else{
            ROS_INFO("arm_srv client response callback error");
        }

        if (srv.response.arm_docking_state == 1){
            ROS_INFO("Docking State: Unsuccessful");
        }
        else if (srv.response.arm_docking_state == 0){
            ROS_INFO("Dock Status: Docking successful");
        }
        else if (srv.response.arm_docking_state == 2){
            ROS_INFO("Dock Status: Arm not lifted");
        }
        else{
            ROS_INFO("arm_srv server callback error");
        }
        
        
    }
    else{
        ROS_INFO("arm_srv client error: int not recognised");
    }


    ros::spin();
    
  }
  else
  {
    ROS_ERROR("Failed to call service arm");
    return 1;
  }

  return 0;
}