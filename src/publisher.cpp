#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"
#include <robot_state_publisher/robot_state_publisher.h>

float wheelRadius = 0.1;
float wLeft,wRight;
const float length_ = 0.8;

void basecallback(const geometry_msgs::Twist::ConstPtr& base)
{
 	float vx = base->linear.x;
    float wth = base->angular.z;
    wRight=((length_*wth)+(2*vx))/(2*wheelRadius);     //get wheel angular velocities from base movement
    wLeft=(-(length_*wth)+(2*vx))/(2*wheelRadius);
}

void wheelscallback(const sensor_msgs::JointState::ConstPtr& wheel)
{
    wRight= wheel->velocity[0];
    wLeft= wheel->velocity[1];
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "Publisher");
  ros::NodeHandle nh_;

  int vel_mult;
  nh_.getParam("vel_mult",vel_mult);

  ros::Publisher vel_pub_;
  ros::Subscriber base_sub_;
  ros::Subscriber wheel_sub_;
  ros::Rate loop_rate(30);

  vel_pub_ = nh_.advertise<sensor_msgs::JointState>("wheel_movement", 1);
  base_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, basecallback);
  wheel_sub_ = nh_.subscribe<sensor_msgs::JointState>("manfred_teleop/joint_state_pub", 1, wheelscallback);

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.velocity.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] ="wheel_servoRjoint";
  joint_state.name[1] ="wheel_servoLjoint";
  while (ros::ok())
	{
		ros::spinOnce();                  //check new messages

        joint_state.velocity[0]=wRight*vel_mult;
        joint_state.velocity[1]=wLeft*vel_mult;

        vel_pub_.publish(joint_state);

        loop_rate.sleep();
    }

  return 0;
}

