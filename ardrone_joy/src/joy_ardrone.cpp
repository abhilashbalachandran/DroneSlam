// done with reference from ros.wiki tutorials for teleop using joystick but modified the code
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

class JoyTwist
{
public:
  JoyTwist();
  ros::NodeHandle nh_;
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void callback(const ros::TimerEvent& event);
  
  int linear, angular;
  double l_scale;
  double a_scale;
  ros::Publisher vel_pub_;
  ros::Publisher takeoff;
  ros::Publisher land;
  ros::Publisher emergency;
  ros::Publisher toggle_pub;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist twist1;
  std_srvs::Empty srv;

  ros::ServiceClient cam_toggle;
};


JoyTwist::JoyTwist()
{
  l_scale = 1;
  a_scale = 1;
  //linear_ = 1;    //since we use axes[linear_] in line 46
  //angular_ = 0;
  //nh_.param("axis_linear", linear_, linear_);
  //nh_.param("axis_angular", angular_, angular_);
  //nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);

  //geometry_msgs:: Twist twist;
 //cam_toggle = nh_.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/togglecam");
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  takeoff = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
  land = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
  emergency = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
  toggle_pub = nh_.advertise<std_msgs::Empty>("/toggle_status",1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTwist::joyCallback, this);
  
  //ros::Timer timer = nh_.createTimer(ros::Duration(0.1), &TeleopTurtle::callback, this);
  
}

void JoyTwist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 

  if(joy->buttons[4])
  { 
    std_msgs::Empty a;
    takeoff.publish(a);
  }

  if(joy->buttons[5])
  { 
    std_msgs::Empty a;
    land.publish(a);

  }

  if(joy->buttons[6])
  {
    std_msgs::Empty a;
    emergency.publish(a);
  }


  //geometry_msgs::Twist twist;
  twist1.angular.z = -1*joy->axes[2];
  //twist1.angular.z = -1*joy->axes[angular_];
  

  twist1.linear.x = l_scale*joy->axes[5];
  twist1.linear.y = l_scale*joy->axes[4];

  twist1.linear.z = joy->buttons[3];

  if(joy->buttons[3])
  {
    twist1.linear.z = joy->buttons[3];
  }

  if(joy->buttons[1])
  {
    twist1.linear.z = -joy->buttons[1];
  }

  if(joy->buttons[7])
  {
    //cam_toggle.call(srv);
    if(ros::service::call("/ardrone/togglecam",srv))
    {
      ROS_INFO("CAM TOGGLE CALLED");
    }
    std_msgs::Empty status_toggle;
    toggle_pub.publish(status_toggle);
  }
  /*if(!(joy->buttons[3]&&joy->buttons[1]))
  {
    twist1.linear.z = 0;
  }*/


  //vel_pub_.publish(twist1);
  //ROS_INFO("I heard [%f] and [%f]",twist.angular.z,twist.linear.x);

}

void JoyTwist::callback(const ros::TimerEvent& event)
{
  //geometry_msgs::Twist twist;
  //twist.angular.z = a_scale_*joy->axes[angular_];
  //twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist1);
  //ROS_INFO("working");
  //twist1.linear.z = 0.0;


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_twist");
  ros::NodeHandle nh;
  JoyTwist Joy_Twist;
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), &JoyTwist::callback, &Joy_Twist);


  //ros::Timer timer = nh.createTimer(ros::Duration(0.1), callback1);
  //ROS_INFO("working");
  ros::spin();
}