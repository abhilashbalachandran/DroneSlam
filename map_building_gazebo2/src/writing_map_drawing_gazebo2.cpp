#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <tf/Transform.h>
//#include <Eigen.h>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <Eigen/Dense>
//#include <tf/LinearMath/btMatrix3x3.h>
using namespace Eigen;

class Plane
{
public:
		 tf::Vector3 p[4];//co-ordinates of points of intersection

		 Plane()
		 {
		//for(int i=0;i<4;i++)				//initializing length values of ar tags
		 	p[0][0] = 0.125; p[0][1] = 0.125;
		 	p[1][0] = -0.125; p[1][1] = 0.125;
		 	p[2][0] = -0.125; p[2][1] = -0.125;
			p[3][0] = 0.125; p[3][1] = -0.125;	 //no of intersection co-ordinates assigned
		}
	};


	std::vector<tf::Transform> visited_tfs;
//std::vector<tf::Transform> unvisited;
	std::vector<tf::Vector3> normals;
	std::vector<tf::Vector3> origins;
	std::vector<Plane> planes;
//Plane planes[10];

	int quadrant_find(tf::Vector3);

	void map_building(tf::Transform tf_cons)
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
				if(n.dot(normals[i])<0.1)			//take only non parallel planes
					index.push_back(i);
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

						if(fabs(A.determinant())>0.01)  	// check if A is non singular
						{	
							ROS_INFO("the three planes intersect");
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
							std::cout<<"Absolute value of X="<<Y<<std::endl;
							//tf::Vector3 temp ;
							tf::Vector3 temp1 = visited_tfs[i].inverse()*X;
							int quad = quadrant_find(temp1);
							//ROS_INFO("QUADRANT = %d",quad);
							planes[i].p[quad] = temp1;
							tf::Vector3 org1 = visited_tfs[i].getOrigin(); 
							//ROS_INFO("ORIGIN ON PLANE 0= %f,%f,%f",org1[0],org1[1],org1[2]);

							//ROS_INFO("intersection point in plane 0 = %f,%f",temp1[0],temp1[1]);

							tf::Vector3 temp2 = visited_tfs[j].inverse()*X;
							quad = quadrant_find(temp2);
							planes[j].p[quad] = temp2;
							tf::Vector3 org2 = visited_tfs[j].getOrigin(); 
							//ROS_INFO("ORIGIN ON PLANE 0= %f,%f,%f",org2[0],org2[1],org2[2]);

							//ROS_INFO("intersection point in plane 1 = %f,%f",temp2[0],temp2[1]);

							tf::Vector3 temp3 = visited_tfs[visited].inverse()*X;
							quad = quadrant_find(temp3);
							planes[visited].p[quad] = temp3;
							tf::Vector3 org3 = visited_tfs[visited].getOrigin(); 
							//ROS_INFO("ORIGIN ON PLANE 0= %f,%f,%f",org3[0],org3[1],org3[2]);

							//ROS_INFO("intersection point in plane 2 = %f,%f",temp3[0],temp3[1]);

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

		int quadrant_find(tf::Vector3 x)
		{
			if (x[0]>0)
			{
				if(x[1]>0)
					return 0;
				else 
					return 3;
			}
			else
			{
				if(x[1]>0)
					return 1;
				else
					return 2;
			}
		}






		int main(int argc, char **argv)

		{
			ros::init(argc,argv,"map_build");
			ros::NodeHandle nh;
			static tf::TransformBroadcaster br;
			std::string transf;
			const double pi = 3.1415926535897;
			tf::Transform tr;
			tr.setOrigin( tf::Vector3(0,0,0));
			tf::Quaternion q;
			q.setRPY(0,0,0);
			tr.setRotation(q);
			br.sendTransform(tf::StampedTransform(tr, ros::Time(0), "world", "child"));
			map_building(tr);

      //tf::Transform tr;
			tr.setOrigin( tf::Vector3(1,0,1));
	  //tf::Quaternion q;
			q.setRPY(0,-pi/2,0);
	  //q.setRotation(tf::Vector3(0,1,0),3.14);
			tr.setRotation(q);
	  //std::cout<<"Tr"<<q[0]<<q[1]<<q[2];
			map_building(tr);


			tr.setOrigin( tf::Vector3(-1,0,1));
	  //tf::Quaternion q;
			q.setRPY(0,pi/2,0);
			tr.setRotation(q);
			map_building(tr);

			tr.setOrigin( tf::Vector3(0,1,1));
	  //tf::Quaternion q;
			q.setRPY(pi/2,0,0);
			tr.setRotation(q);
			map_building(tr);


			 ROS_INFO("Point values are = %f, %f, then = , %f, %f , = then %f,%f and = %f,%f",planes[0].p[0][0],planes[0].p[0][1], planes[0].p[1][0],planes[0].p[1][1] , planes[0].p[2][0],planes[0].p[2][1], planes[0].p[3][0],planes[0].p[3][1]);

			 ROS_INFO("Point values are = %f, %f, then = , %f, %f , = then %f,%f and = %f,%f",planes[1].p[0][0],planes[1].p[0][1], planes[1].p[1][0],planes[1].p[1][1] , planes[1].p[2][0],planes[1].p[2][1], planes[1].p[3][0],planes[1].p[3][1]);

			 ROS_INFO("Point values are = %f, %f, then = , %f, %f , = then %f,%f and = %f,%f",planes[2].p[0][0],planes[2].p[0][1], planes[2].p[1][0],planes[2].p[1][1] , planes[2].p[2][0],planes[2].p[2][1], planes[2].p[3][0],planes[2].p[3][1]);

			 ROS_INFO("Point values are = %f, %f, then = , %f, %f , = then %f,%f and = %f,%f",planes[3].p[0][0],planes[3].p[0][1], planes[3].p[1][0],planes[3].p[1][1] , planes[3].p[2][0],planes[3].p[2][1], planes[3].p[3][0],planes[3].p[3][1]);

			ros::spin();
			return 0;
		}
