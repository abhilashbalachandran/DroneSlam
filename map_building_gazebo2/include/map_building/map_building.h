#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMArkers.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class MapBuilding
{
	struct Wall
	{
		int id;
		tf::Transform world_tf;
		double plus_l_ex;
		double minus_l_ex;
		double plus_b_ex;
		double minus_b_ex;
	}

public:
	MapBuilding();
	void ar_marker_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr marker_msg); ///<< add visited frames to list
	void tf_timer_callback(const ros::TimerEvent &event); ///<< check for tf between visted frames and add them to tf list	

private:
	void updateMap()// decide arguments; ////<< update map based on new markers found
	void makeWall(Wall wall) // decide arguments ////<< add a wall to the map
	void makeCube() // decide arguments ////<< add a cube to the map

private:
	ros::NodeHandle nh;
	ros::Subscriber ar_track_sub;
	ros::Publisher marker_pub;
	ros::Timer tf_timer_event;

	int base_frame_id;
	std::vector<int> visited_frames; // list of visited frames
	std::vector<tf::Transform> tf_list; // list of tf w.r.t base frame
	std::vector<double[3]> v;

};

MapBuilding::MapBuilding()
{
	//TODO
}

void MapBuilding::makeWall(Wall wall)
{
	visualization_msgs::Marker points;

  //specify header and type
	points.header.frame_id = "ar_marker_" + std::to_string(base_frame_id);
	points.header.stamp = ros::Time::now();
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;

  // scale and color
  double scale = 0.02
	points.scale.x = scale;
	points.scale.y = scale;
	points.color.g = 1.0;
	points.color.a = 1.0;

	int length_minus;
	int length_plus;
	int width_minus;
	int width_plus;

	if(wall.minus_l_ex == -1)
		length_minus = round(10/scale);
	else
		length_minus = round(minus_l_ex/scale);

	if(wall.plus_l_ex == -1)
		length_plus = round(10/scale);
	else
		length_plus = round(plus_l_ex/scale);

	if(wall.minus_b_ex == -1)
		width_minus = round(10/scale);
	else
		width_minus = round(minus_b_ex/scale);

	if(wall.plus_b_ex == -1)
		width_plus = round(10/scale);
	else
		width_plus = round(plus_b_ex/scale);

		for(int i= -length_minus; i < length_plus; i++)
			for(int j= width_minus; j < width_plus; j++)
			{
				geometry_msgs::Point p;
				p.x = scale*i;
				p.z = scale*j;

				points.points.push_back(p);
			}

			marker_pub.publish(points);

		}

