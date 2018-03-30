#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class CreateOccupancyGrid
{
public:
	ros::NodeHandle nh;
	ros::Publisher grid_pub;
	ros::Subscriber grid_sub;
	nav_msgs::OccupancyGrid map;

public:
	CreateOccupancyGrid();
};

CreateOccupancyGrid::CreateOccupancyGrid()
{
	grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid",10);
	map.info.map_load_time = ros::Time::now();
	map.info.resolution = 0.1;
	map.info.width = 10;
	map.info.height = 10;
	map.data.resize(100);


	ROS_INFO("Initializing data....");
	
	for(int i=0;i>100;i++)
		map.data[i] = -1;

	ROS_INFO("Initialized data.....");
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"create_occupancy_grid");

	CreateOccupancyGrid occupancy_grid_instance;
	ROS_INFO("created object");
	ros::Duration(2.0).sleep();

	while(ros::ok())
	{
		for(int i=2; i < 8; i++)
		{
			for(int j=4; j < 6; j++)
			{
				occupancy_grid_instance.map.data.at(i*10+j) = 100;
				// ROS_INFO("%ith data point added...",i+j);
			}
		}

		// ROS_INFO("data added...");
		occupancy_grid_instance.grid_pub.publish(occupancy_grid_instance.map);
	}
	ros::spin();

	return 0;
}