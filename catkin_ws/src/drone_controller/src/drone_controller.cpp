 /*
 * Originally from: https://dev.px4.io/ros-mavros-offboard.html
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/Altitude.h> // mavros_msgs::Altitude

#include "pid.h"


class PID;

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
 current_state = *msg;
}

geometry_msgs::Point point;
void point_cb(const geometry_msgs::Point::ConstPtr& msg){
  point = *msg;
}

mavros_msgs::Altitude altitude;
void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){
  altitude = *msg;
}

int main(int argc, char **argv)
{

  PID pid_position_x = PID(0.1, 1, -1, 0.1, 0.01, 1.0);

  PID pid_height = PID(0.1, 0.5, -0.5, 1.0, 0.1, 0.01);

  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                              ("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                      ("mavros/set_mode");
  ros::Subscriber point_sub = nh.subscribe<geometry_msgs::Point>
                                          ("/location", 10, point_cb);

  ros::Subscriber alt_pos = nh.subscribe<mavros_msgs::Altitude>
    ("mavros/altitude", 10, altitude_cb);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  // wait for FCU connection
  while(ros::ok() && current_state.connected){
  ros::spinOnce();
  rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 5;
 //send a few setpoints before starting, or else you won't be able to
//switch to offboard mode
  for(int i = 100; ros::ok() && i > 0; --i){
  local_pos_pub.publish(pose);
  ros::spinOnce();
  rate.sleep();
  }
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();
  while(ros::ok()){
  if( current_state.mode != "OFFBOARD" &&
   (ros::Time::now() - last_request > ros::Duration(5.0))){
   if( set_mode_client.call(offb_set_mode) &&
   offb_set_mode.response.mode_sent){
    ROS_INFO("Offboard enabled");
   }
  last_request = ros::Time::now();
  } else {
  if( !current_state.armed &&
    (ros::Time::now() - last_request > ros::Duration(5.0))){
     if( arming_client.call(arm_cmd) &&
     arm_cmd.response.success){
     ROS_INFO("Vehicle armed");
   }
   last_request = ros::Time::now();
  }
  }
  cout << "altitude: " << altitude.local << endl;
  cout << "X: " << point.x << "Y: " << point.y <<endl;
  double x_pos_error = pid_position_x.calculate(-point.x, pose.pose.position.x);
  double height_error = pid_height.calculate(5, altitude.local);
  cout << "x_pos_error" << x_pos_error << endl;
  if(altitude.local > 3)
    pose.pose.position.x = x_pos_error;
  local_pos_pub.publish(pose);
  ros::spinOnce();
  rate.sleep();
  }
  return 0;
}
