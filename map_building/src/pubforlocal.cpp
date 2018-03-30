#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv)
{
	ros::init(argc,argv,"fake_localization");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);

	geometry_msgs::PoseWithCovarianceStamped pose_initial;
	pose_initial.pose.pose.position.x = 1.0;
	pose_initial.pose.pose.position.y = 1.0;
	pose_initial.pose.pose.position.z = 0.0;

	pose_initial.pose.pose.orientation.x = 0.0;
	pose_initial.pose.pose.orientation.y = 0.0;
	pose_initial.pose.pose.orientation.z = 0.0;
	pose_initial.pose.pose.orientation.w = 1.0;

	tf::TransformBroadcaster br;
	tf::Transform edumip_world_transform;
	pose_initial.header.frame_id = "map";
	 edumip_world_transform.setOrigin(tf::Vector3(1, 0, 0) );
  tf::Quaternion q;
  q.setRPY(0, 0,0);
  edumip_world_transform.setRotation(q);
  
	//while(ros::ok())
	//{
		br.sendTransform(tf::StampedTransform(edumip_world_transform, ros::Time::now(), "map", "edumip_body"));
		//ros::Duration(0.2).sleep();
		pub.publish(pose_initial);
		//ros::Duration(0.2).sleep();
	//}
	



	return 0;


}