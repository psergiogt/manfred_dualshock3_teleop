#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <sensor_msgs/JointState.h>

class ManfredTeleop
 {
 public:
   ManfredTeleop();
 
 private:
   void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
   void publish();
 
   ros::NodeHandle ph_, nh_;
 
   int linear_, angular_, r2_axis_,l2_axis_,linearL_,linearR_;
   double l_scale_, a_scale_,l_scaleL_,l_scaleR_;
   ros::Publisher base_pub_;
   ros::Publisher wheel_pub_;
   ros::Subscriber joy_sub_;
 
   geometry_msgs::Twist last_twist_published_;
   sensor_msgs::JointState last_joint_published_;
   boost::mutex publish_mutex_;
   bool r2_pressed_;
   bool l2_pressed_;
   bool zero_twist_published_;
   ros::Timer timer_;
 
 };
 
 ManfredTeleop::ManfredTeleop():
   ph_("~"),
   linear_(1),
   angular_(0),
   r2_axis_(9),
   l2_axis_(8),
   l_scale_(0.3),
   a_scale_(0.9),
   linearL_(1),
   linearR_(3),
   l_scaleL_(1),
   l_scaleR_(1)
 {
   ph_.param("axis_linear", linear_, linear_);
   ph_.param("axis_angular", angular_, angular_);
   ph_.param("axis_r2", r2_axis_, r2_axis_);
   ph_.param("axis_l2", l2_axis_, l2_axis_);
   ph_.param("scale_linear", l_scale_, l_scale_);
   ph_.param("scale_angular", a_scale_, a_scale_);
   ph_.param("axis_linearL", linearL_, linearL_);
   ph_.param("axis_linearR", linearR_, linearR_);
   ph_.param("scale_linearL", l_scaleL_, l_scaleL_);
   ph_.param("scale_linearR", l_scaleR_, l_scaleR_);
 
   r2_pressed_ = false;
   l2_pressed_ = false;
   zero_twist_published_ = false;
 
   base_pub_ = ph_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
   wheel_pub_ = ph_.advertise<sensor_msgs::JointState>("joint_state_pub", 1);
   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ManfredTeleop::joyCallback, this);
 
   timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&ManfredTeleop::publish, this));
 }
 
 void ManfredTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
 { 
   geometry_msgs::Twist vel;
   sensor_msgs::JointState joint_state;
   joint_state.header.stamp = ros::Time::now();
   joint_state.name.resize(2);
   joint_state.velocity.resize(2);
   joint_state.position.resize(2);
   joint_state.name[0] ="wheel_servoRjoint";
   joint_state.name[1] ="wheel_servoLjoint";
   vel.angular.z = a_scale_*joy->axes[angular_];
   vel.linear.x = l_scale_*joy->axes[linear_];
   joint_state.velocity[0] = l_scaleR_*joy->axes[linearR_];
   joint_state.velocity[1] = l_scaleL_*joy->axes[linearL_];
   last_twist_published_ = vel;
   last_joint_published_ = joint_state;
   r2_pressed_ = joy->buttons[r2_axis_];
   l2_pressed_ = joy->buttons[l2_axis_];
 }
 
 void ManfredTeleop::publish()
 {
   boost::mutex::scoped_lock lock(publish_mutex_);
 
   if (r2_pressed_ && !l2_pressed_)
   {
     base_pub_.publish(last_twist_published_);
     zero_twist_published_=false;
   }
   else if (r2_pressed_ && l2_pressed_)
   {
     base_pub_.publish(*new geometry_msgs::Twist());
     zero_twist_published_=true;
   }
   else if (l2_pressed_ && !r2_pressed_)
   {
     wheel_pub_.publish(last_joint_published_);
     zero_twist_published_=false;
   }
   else if(!(r2_pressed_ || l2_pressed_) && !zero_twist_published_)
   {
     base_pub_.publish(*new geometry_msgs::Twist());
     zero_twist_published_=true;
   }
 }
 
 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "manfred_teleop_joy");
   ManfredTeleop Manfred_teleop;
 
   ros::spin();
 }
