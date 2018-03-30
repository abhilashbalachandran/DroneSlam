	#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "edumip_msgs/EduMipState.h"
#include <tf/transform_datatypes.h>

#include <tf/transform_broadcaster.h>


class EduMip
{
	nav_msgs::Odometry odom;
	ros::NodeHandle nh;
	ros::Publisher nav_pub;
	tf::Transform edumip_world_transform;
	 tf::TransformBroadcaster br;
	 	tf::Transform x; 


public:
	EduMip()
	{
		nav_pub = nh.advertise<nav_msgs::Odometry>("edumip/odom",10);
	}
	void edumipcallback(const edumip_msgs::EduMipState::ConstPtr &edumip_state_msg);
	void tf_pub_timer(const ros::TimerEvent &event);
	
};

void EduMip::edumipcallback(const edumip_msgs::EduMipState::ConstPtr &edumip_state_msg)
{   //ROS_INFO("In callback");

	odom.header.frame_id="map";
	odom.child_frame_id="edumip_body";


	odom.header.stamp = ros::Time::now();
	odom.pose.pose.position.y= edumip_state_msg->body_frame_northing;
	odom.pose.pose.position.x = -edumip_state_msg->body_frame_easting;
	odom.pose.pose.position.z = 0.034;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(edumip_state_msg->theta,0,edumip_state_msg->body_frame_heading);
	nav_pub.publish(odom);
	
	x.setOrigin(tf::Vector3(1,1,0));
	x.setRotation(tf::Quaternion(0,0,0,1));





	edumip_world_transform.setOrigin(tf::Vector3(edumip_state_msg->body_frame_northing, -edumip_state_msg->body_frame_easting, 0.034) );
  tf::Quaternion q;
  q.setRPY(0, edumip_state_msg->theta, -edumip_state_msg->body_frame_heading);
  edumip_world_transform.setRotation(q);
	//ROS_INFO("odom ran");

}

void EduMip::tf_pub_timer(const ros::TimerEvent &event)
{
	br.sendTransform(tf::StampedTransform(edumip_world_transform.inverse()*x, ros::Time::now(),"edumip_body", "map"));

}




int main(int argc, char  **argv)
{
	
	ros::init(argc,argv,"edumip");
	ros::NodeHandle nh;
	EduMip edumip;
	ros::Subscriber edumip_subscribe = nh.subscribe("edumip/state",60,&EduMip::edumipcallback,&edumip);
	ros::Timer tf_timer = nh.createTimer(ros::Duration(0.1),&EduMip::tf_pub_timer,&edumip);
	ros::spin();
	return 0;
	
}