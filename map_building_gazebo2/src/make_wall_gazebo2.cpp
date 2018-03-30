#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <nav_msgs/OccupancyGrid.h>

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <Eigen/Dense>

using namespace Eigen;

class Plane
{
public:
	tf::Vector3 p[4];//co-ordinates of points of intersection

	Plane()
	{
//for(int i=0;i<4;i++)				//initializing length values of ar tags
		p[0][0] = 0.125; p[0][1] = 0.125; p[0][2] = 0.0;
		p[1][0] = -0.125; p[1][1] = 0.125; p[1][2] = 0.0;
		p[2][0] = -0.125; p[2][1] = -0.125; p[2][2] = 0.0;
		p[3][0] = 0.125; p[3][1] = -0.125;	p[3][2] = 0.0; //no of intersection co-ordinates assigned
	}
};


class MakeWall
{
public:
	ros::NodeHandle nh;
	ros::Publisher marker_pub;
	ros::Subscriber ar_track_alvar_sub;
	tf::TransformListener* listener;

	ros::Publisher grid_pub;
	ros::Subscriber grid_sub;
	ros::Timer grid_timer;
	nav_msgs::OccupancyGrid map;


	std::vector<int> visited_frames;
	int base_id;
	std::string base_frame;

	visualization_msgs::Marker points1;

	std::vector<tf::Transform> visited_tfs;
	std::vector<tf::Vector3> normals;
	std::vector<tf::Vector3> origins;
	std::vector<Plane> planes;

public:
	MakeWall();
	void make_marker(tf::Transform transform, 
		tf::Vector3 p1,
		tf::Vector3 p2,
		tf::Vector3 p3,
		tf::Vector3 p4);

	void ar_track_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr alvar_msg);
	void map_building(tf::Transform tf_cons);
	void quadrant_find(tf::Vector3 x,int i);
	void updateOccupancyGrid();
	void publishGrid(const ros::TimerEvent &event);

};

MakeWall::MakeWall()
{
	marker_pub = nh.advertise<visualization_msgs::Marker>("my_marker",10);
	ar_track_alvar_sub = nh.subscribe("/ar_pose_marker",10,&MakeWall::ar_track_Callback,this);
	grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map",10);
	listener = new tf::TransformListener(ros::Duration(1000.0));
	grid_timer = nh.createTimer(ros::Duration(0.1),&MakeWall::publishGrid,this);

    // Marker Array Initialization
	points1.header.frame_id = "nav";
	points1.header.stamp = ros::Time::now();
	points1.pose.orientation.w = 1.0;
	points1.id = 0;
	points1.type = visualization_msgs::Marker::POINTS;
	points1.scale.x = 0.02;
	points1.scale.y = 0.02;

	//Occupancy Grid initialization
	map.info.map_load_time = ros::Time::now();
	map.info.resolution = 0.01;
	map.info.width = 2000;
	map.info.height = 2000;
	map.info.origin.position.x = -10.0;
	map.info.origin.position.y = -10.0;
	map.info.origin.orientation.w = 1.0;
	map.data.resize(2000*2000);

}

void MakeWall::ar_track_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr alvar_msg)
{

	//ROS_INFO("Marker found");
	if(alvar_msg->markers.size() != 0)

	{
		if(visited_frames.empty())
		{
			ROS_INFO("Adding base frame %i",alvar_msg->markers[0].id);
			visited_frames.push_back(alvar_msg->markers[0].id);
			base_id = alvar_msg->markers[0].id;
			base_frame = "ar_marker_" + std::to_string(base_id);

			tf::Transform transform;
			transform.setOrigin( tf::Vector3(0.0 ,0.0, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, 0);
			transform.setRotation(q);

			map_building(transform);
			make_marker(transform,planes[0].p[0],planes[0].p[1],planes[0].p[2],planes[0].p[3]);
			tf::StampedTransform base_tf = tf::StampedTransform(transform,ros::Time::now(),"nav",base_frame);
			ROS_INFO("made base marker");
			//make_marker(base_tf);

		}

		else if(find(visited_frames.begin(),visited_frames.end(),alvar_msg->markers[0].id) == visited_frames.end())
		{
			tf::StampedTransform transform;
			std::string new_frame = "ar_marker_" + std::to_string(alvar_msg->markers[0].id);
			ros::Time now = ros::Time::now();
			try
			{
				//listener->waitForTransform(base_frame,new_frame,now,ros::Duration(3.0));
				listener->lookupTransform(base_frame,new_frame,ros::Time(0),transform);
				tf::Vector3 origin = transform.getOrigin();

				//ROS_INFO("TF in base frame = %f %f %f",origin[0],origin[1],origin[2]);
				visited_frames.push_back(alvar_msg->markers[0].id);
				//ROS_INFO("Adding new frame : frame id %i",alvar_msg->markers[0].id);

				map_building(transform);

				for(int i=0; i < visited_tfs.size(); i++)
				{
					tf::Vector3 p1 = planes[i].p[0];
					tf::Vector3 p2 = planes[i].p[1];
					tf::Vector3 p3 = planes[i].p[2];
					tf::Vector3 p4 = planes[i].p[3];
					tf::Transform tf_temp = visited_tfs[i];
					ROS_INFO("p1  in plane %i = %f,%f",i,p1[0],p1[1]);
					ROS_INFO("p2 in plane %i = %f,%f",i,p2[0],p2[1]);
					ROS_INFO("p3 in plane %i = %f,%f",i,p3[0],p3[1]);
					ROS_INFO("p4 in plane %i = %f,%f",i,p4[0],p4[1]);

					make_marker(tf_temp,p1,p2,p3,p4);
					ROS_INFO("made marker");
				}

				updateOccupancyGrid();
				//ROS_INFO("frames found = %i",visited_tfs.size());

			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		}
	}
}

void MakeWall::publishGrid(const ros::TimerEvent &event)
{
	grid_pub.publish(map);
}

void MakeWall::make_marker(tf::Transform transform, 
	tf::Vector3 p1,
	tf::Vector3 p2,
	tf::Vector3 p3,
	tf::Vector3 p4)
//tf::Vector3 p1 = plane[i].p[0];
{

	// Transform points in local frame
	/*
  tf::Vector3 p1 = transform*point1;
  tf::Vector3 p2 = transform*point2;
  tf::Vector3 p3 = transform*point3;
  tf::Vector3 p4 = transform*point4;

  */

  // Do not continue if points are not on the plane, i.e. 
	if (fabs(p1[2]) > 0.01)
	{
		//ROS_INFO("p1 is not on the plane");
		return;
	}
	if (fabs(p2[2]) > 0.01)
	{
		//ROS_INFO("p2 is not on the plane");
		return;
	}
	if (fabs(p3[2]) > 0.01)
	{
		//ROS_INFO("p3 is not on the plane");
		return;
	}
	if (fabs(p4[2]) > 0.01)
	{
		//ROS_INFO("p4 is not on the plane");
		return;
	}

  //Find diagonally opposite point to p1
	tf::Vector3 o_p1;
	if(fabs(p1[0] - p2[0]) > 0.1 && fabs(p1[1] - p2[1]) > 0.1)
	{
		o_p1 = p2;
		//ROS_INFO("p2 is opposite p1");
	}
	else if(fabs(p1[0] - p3[0]) > 0.1 && fabs(p1[1] - p3[1]) > 0.1)
	{
		o_p1 = p3;
		//ROS_INFO("p3 is opposite p1");
	}
	else if(fabs(p1[0] - p4[0]) > 0.1 && fabs(p1[1] - p4[1]) > 0.1)
	{
		o_p1 = p4;
		//ROS_INFO("p4 is opposite p1");
	}

  //Find extents on both sides
	double plus_x,plus_y,minus_x,minus_y;

	if( p1[0] > 0.0)
	{
		plus_x = p1[0];
		minus_x = o_p1[0];

	}
	else
	{
		minus_x = p1[0];
		plus_x = o_p1[0];
	}

	if( p1[1] > 0.0)
	{
		plus_y = p1[1];
		minus_y = o_p1[1];

	}
	else
	{
		minus_y = p1[1];
		plus_y = o_p1[1];
	}

	//ROS_INFO("extents plus_x = %f, plus_y = %f, minus_x = %f, minus_y = %f",
		//plus_x, plus_y, minus_x, minus_y);

	int p_x = round(plus_x/0.02);
	int p_y = round(plus_y/0.02);
	int m_x = round(minus_x/0.02);
	int m_y = round(minus_y/0.02);


	for(int i = m_x; i < p_x; i++)
	{
		for(int j = m_y; j < p_y; j++)
		{ 
			tf::Vector3 next_point = transform*tf::Vector3(0.02*i , 0.02*j, 0);
			geometry_msgs::Point p;
			p.x = next_point[0];
			p.y = next_point[1];
			p.z = next_point[2];

			std_msgs::ColorRGBA c;
			c.r = 1.0;
			c.a = 1.0;
			points1.points.push_back(p);
			points1.colors.push_back(c);
				// ROS_INFO("pushed back : %f %f %f",p.x,p.y,p.z);
		}
	}

	marker_pub.publish(points1);

}

void MakeWall::map_building(tf::Transform tf_cons)
{
	//adding to visited
	visited_tfs.push_back(tf_cons);
	Plane plane_cons;
	planes.push_back(plane_cons);
	//visited_tf.push_back(tf_cons);	
	//VectorXd n(3);
	tf::Transform temp_tf;
	tf::Quaternion q = tf_cons.getRotation();
	temp_tf.setOrigin(tf::Vector3(0,0,0));
	temp_tf.setRotation(q);
	tf::Vector3 n = temp_tf*(tf::Vector3(0,0,1));

	tf::Vector3 e = tf_cons*(tf::Vector3(0,0,0));

	normals.push_back(n);
	origins.push_back(e);


	//if 3 visited planes 
	//write A,B,C
	//B = inv(A)*C
	//check if A is singular
	std::vector<int> index;
	int visited = visited_tfs.size()-1;
	//ROS_INFO("size of visited = %d",visited);
	//ROS_INFO("normal = %f, %f, %f",n[0],n[1],n[2]);
	//ROS_INFO("origin = %f, %f, %f",e[0],e[1],e[2]);
	//ROS_INFO("normal = %f, %f, %f",normals[visited-1][0],normals[visited-1][1],normals[visited-1][2]);
	//ROS_INFO("origin = %f, %f, %f",origins[visited-1][0],origins[visited-1][1],origins[visited-1][2]);

	//3 or more visited frames
	if(visited_tfs.size()>2)
	{
		MatrixXd A(3,3);
		Vector3d B;
		//n1*(e1 - x) + n2*(e2 - y) + n3*(e3 - z);
		//n1*(x-e1) + n2*(y-e1) + n3*(z-e3);
		//find intersecting planes
		for (int i=0;i<visited_tfs.size();i++)
		{
			if(n.dot(normals[i])<0.1)
							//take only non parallel planes
			{
				index.push_back(i);
			}
		}


		//check intersection of current plane with pairs of other planes
		for(int i=0;i<index.size();i++)
		{
			for(int j=i;j<index.size();j++)
			{
				A << normals[i][0], normals[i][1], normals[i][2],
				normals[j][0], normals[j][1], normals[j][2],
				normals[visited][0], normals[visited][1], normals[visited][2];


				B << normals[i].dot(origins[i]), normals[j].dot(origins[j]), normals[visited].dot(origins[visited]);

				if(fabs(A.determinant()) > 0.1)  	// check if A is non singular
				{	
					//ROS_INFO("the three planes intersect");
					//std::cout<<"A MATRIX = "<<std::endl<<A<<std::endl;
					//std::cout<<"B Matrix = "<<std::endl<<B<<std::endl;
							//Vector3d Y = A.ldlt().solve(B);
					Vector3d Y = A.inverse()*B;
					//std::cout<<"A INVERSE = "<<std::endl<<Y<<std::endl;
					tf::Vector3 X;
					X[0] = Y[0];
					X[1] = Y[1];
					X[2] = Y[2];

							//Vector3d X = A.inverse()*B;
							//ROS_INFO("intersection point in ground plane = %f,%f,%f",X[0],X[1],Y[1]);
							//now transform the point in their respective co-ordinate frames
					tf::Vector3 temp ;
					temp = visited_tfs[i].inverse()*X;
					quadrant_find(temp,i);
					//ROS_INFO("QUADRANT = %d",quad);
					//planes[i].p[quad] = temp;

				//	ROS_INFO("intersection point in plane 0 = %f,%f",temp[0],temp[1]);


					temp = visited_tfs[j].inverse()*X;
					quadrant_find(temp,j);
					//planes[j].p[quad] = temp;

				//	ROS_INFO("intersection point in plane 1 = %f,%f",temp[0],temp[1]);


					temp = visited_tfs[visited].inverse()*X;
					quadrant_find(temp,visited);
					//planes[visited].p[quad] = temp;
				//	ROS_INFO("intersection point in plane 2 = %f,%f",temp[0],temp[1]);



							//planes[i].p[planes[i].intersect] = visited_tfs[i]*X;(planes[i].intersect)++;
							//planes[j].p[planes[j].intersect] = visited_tfs[j]*X;(planes[j].intersect)++;
							//planes[visited].p[planes[visited].intersect] = visited_tfs[visited]*X;(planes[visited].intersect)++;
							// tf::Vector3 tsd = visited_tfs[visited-1]*X;
							// j = visited-1;
							// ROS_INFO("output = %f,%f,%f",planes[j].p[planes[j].intersect-1][0],planes[j].p[planes[j].intersect-1][1],planes[j].p[planes[j].intersect-1][2]);
							// ROS_INFO("intersection index = %d",planes[j].intersect);
				}

			}
		}


	}

}

void MakeWall::quadrant_find(tf::Vector3 x,int i)
{
	if (x[0]>0)
	{
		if(x[1]>0)
		{ 
			planes[i].p[1][1] = x[1];
			planes[i].p[3][0] = x[0];
			planes[i].p[0] = x;
		}
		else 
		{ 
			planes[i].p[0][0] = x[0];
			planes[i].p[2][1] = x[1];
			planes[i].p[3] = x;
		}
	}
	else
	{
		if(x[1]>0)
		{ 
			planes[i].p[0][0] = x[1];
			planes[i].p[2][0] = x[0];
			planes[i].p[1] = x;
		}
		else
		{ 
			planes[i].p[1][0] = x[0];
			planes[i].p[3][1] = x[1];
			planes[i].p[2] = x;
		}
	}
}

void MakeWall::updateOccupancyGrid()
{ 
	for(int k=0; k<2000; k++)
	{
		for(int m=0; m<2000; m++)
		{
			map.data.at(k*2000 + m) = 0;
		}
	}
	
	std::vector<tf::Transform> obstacle_tfs;

	for(int i=0; i < visited_tfs.size(); i++)
	{
		tf::Vector3 origin_temp = visited_tfs[i].getOrigin();
		if(fabs(origin_temp[2]) < 0.01)
		{
			ROS_INFO("frame %i has z=0. Adding to grid....",i);
			double origin_x = origin_temp[0] + 10.0;
			double origin_y = origin_temp[1] + 10.0;
			for(int x=-13; x < 13; x++)
			{
				for(int y=-13; y < 13; y++)
				{
					map.data.at((round(origin_x/0.01) + x) + (round(origin_y/0.01) + y)*2000) = 100;
				}
			}
		}
	}

	double p1_x = planes[0].p[0][0] + 10.0;
	double p2_x = planes[0].p[1][0] + 10.0;
	double p3_x = planes[0].p[2][0] + 10.0;
	double p4_x = planes[0].p[3][0] + 10.0;

	double p1_y = planes[0].p[0][1] + 10.0;
	double p2_y = planes[0].p[1][1] + 10.0;
	double p3_y = planes[0].p[2][1] + 10.0;
	double p4_y = planes[0].p[3][1] + 10.0;

	ROS_INFO("p1_y = %f, p2_y = %f", p1_y, p2_y);
	ROS_INFO("p2_x = %f, p3_x = %f", p2_x, p3_x);
	ROS_INFO("p3_y = %f, p4_y = %f", p3_y, p4_y);
	ROS_INFO("p1_x = %f, p4_x = %f", p1_x, p4_x);


	for(int x = 0; x < abs(round((p1_x - p2_x)/0.01)); x++)
		map.data.at((round(p2_x/0.01) + x) + round(p2_y/0.01)*2000) = 100;

	for(int y = 0; y < abs(round((p2_y - p3_y)/0.01)); y++)
		map.data.at(round(p3_x/0.01) + (round(p3_y/0.01) + y)*2000) = 100;

	for(int x = 0; x < abs(round((p4_x - p3_x)/0.01)); x++)
		map.data.at((round(p3_x/0.01) + x) + round(p3_y/0.01)*2000) = 100;

	for(int y = 0; y < abs(round((p1_y - p4_y)/0.01)); y++)
		map.data.at(round(p4_x/0.01) + (round(p4_y/0.01) + y)*2000) = 100;

}




int main( int argc, char** argv )
{
	ros::init(argc, argv, "make_wall_gazebo2");
	ROS_INFO("Node Started");

	MakeWall make_wall_instance;
	ROS_INFO("Created Instance:");
	ros::spin();

	return 0;

}





