#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <random>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
using namespace std ;
using namespace Eigen;

typedef Eigen::Vector3d V3d;

//IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

void GenerateGaps_dynamic(V3d cur, std::vector<mid_info> &result)
{
	for(size_t i = 0; i < obs_num-1; i++)
	{
		for(size_t j = i+1; j < obs_num; j++)
		{
			mid_info gap = computetan(sphere_centers[i], sphere_radius[i]+to_obs,
									sphere_centers[j], sphere_radius[j]+to_obs, cur);

			result.push_back(gap);
		}
	}
	std::sort(result.begin(), result.end(),Mid_Greater);

}
mid_info computetan(const V3d& c1, const double& r1, const V3d& c2, cosnt double& r2, const V3d &cur)
{
	V3d cur2c1_dir = (c1-cur).normalized();
	V3d cur2c2_dir = (c2-cur).normalized();
	V3d axisz = cur2c1_dir.cross(cur2c2_dir);


	//the tan point on c1
	V3d c1_yaxis = axisz.cross(cur2c1_dir);
	
	double c1_angle = asin(r1/((c1-cur).norm()));
	double c1len = sqrt((c1-cur).dot(c1-cur) - r1*r1);
	Matrix3d m;
	//cout << cur2c1_dir << endl;
	//cout << endl<< axisz << endl;
	m = AngleAxisd(c1_angle, axisz.normalized());
	V3d c1tan_framec1 = m*cur2c1_dir*c1len;
	V3d c1tan = c1tan_framec1 + cur;
	
	//the tan point on c2
	double c2_angle = asin(r2/((c2-cur).norm()));
	double c2len = sqrt((c2-cur).dot(c2-cur) - r2*r2);
	m = AngleAxisd(-c2_angle, axisz.normalized());
	V3d c2tan_framec1 = m*cur2c2_dir*c2len;
	V3d c2tan = c2tan_framec1 + cur;
	

	//check
	/*
	cout << "!!!!!!!!!!!!!!!!"<<endl;
	
	cout <<" checking z axis"<<endl;
	//V3d z1 = ((c1tan-cur).cross(c1-cur)).normalized();
	V3d z = ((c1tan-cur).cross((c2tan-cur))).normalized();
	cout << z << endl;
	//cout << endl<<z1<<endl;
	cout << axisz.normalized() <<endl;

	cout << "checking length" << endl;
	V3d cur2tan1 =  c1tan - cur;
	double r1_check = sqrt((c1-cur).dot(c1-cur)-(c1tan-cur).dot(c1tan-cur));
	cout << "r 1 : "<< r1_check << endl;
	V3d cur2tan2 =  c2tan - cur;
	double r2_check = sqrt((c2-cur).dot(c2-cur)-(c2tan-cur).dot(c2tan-cur));
	cout << "r 2 : "<< r2_check << endl;
	*/	

	// checking if the mid point is free
	V3d mid = (c1tan+c2tan)*0.5;
	double mid_dis = (c1tan-c2tan).norm();
	return mid_info(mid,mid_dis,-1);

}

bool plan_daynamic(V3d start)
{
	V3d cur = start;
	std::vector<mid_info> GapSet;
	std::vector<double> joint_values;
	initpTraj.clear();

	// start point 
	robot_state_.copyJointGroupPositions(joint_model_group_, joint_values);
  	initpTraj.push_back(joint_values);

  	// for IK solver : initial value && the solution 
  	moveit_msgs::RobotState moveit_init;
	moveit_msgs::RobotState moveit_res;
	// copy the start state 
  	moveit::core::robotStateToRobotStateMsg(robot_state_,moveit_init);
  	int stepnum = 0;
	while(cur!=goal)
	{
		std::cout << "!!!!!!!!!!!!!!!one step !!!!!!!:  " << stepnum << std::endl;
		GapSet.clear();
		GenerateGaps_dynamic(cur, GapSet);

		if(GapSet.size()<1)
		{
			std::cout << "plan fail!! Gap set is empty!!" <<std::endl;
			return false;
		}
		bool foundone = false;

		
		for(int j = 0; j < GapSet.size(); j++)
		{
			const mid_info& onemid = GapSet[j];
			// check if near
			if((onemid.pos - goal).norm() < (cur - goal).norm())
			{
				// check if free
				if(checkSegment(cur, onemid.pos))
				{
					//check if IK solution exists
					
  
					if(checkCartPoseCollision(onemid.pos, moveit_init,moveit_res))
					{
						moveit::core::robotStateMsgToRobotState(moveit_res, robot_state_);
           				robot_state_.copyJointGroupPositions(joint_model_group, joint_values);
						visual_tools_.publishRobotState(robot_state_, rviz_visual_tools::GREEN);
						moveit_rs = moveit_res;
						initpTraj.push_back(joint_values);
						cur = onemid.pos;
						foundone = true;
						break;
					}
					else
					{
						continue;
					}
				}
				else
				{
					//check next mid point
					continue;
				}
				
			}
			else
			{ // check next mid point
				continue;
			}
		}
		if(!foundone)
		{
			ROS_ERROR_STREAM("Not found any available mid point.");
			return false;
		}
		stepnum ++;

	}
}

int main()
{
	

	V3d startpoint(2,2,7);

	vector<V3d> centers;

	vector<double> radius;

	centers.push_back(V3d(3,5,7));
	radius.push_back(1.2);

	centers.push_back(V3d(5,8,4));
	radius.push_back(2);
	
	//m = Eigen::AngleAxisf(0.25*M_PI, startpoint.normalized());
	//m = Eigen::AngleAxisf(0.25*M_PI, Eigen::Vector3f::UnitX())
	//  * Eigen::AngleAxisf(0.5*M_PI,  Eigen::Vector3f::UnitY())
	//  * Eigen::AngleAxisf(0.33*M_PI, Eigen::Vector3f::UnitZ());
	//V3d c1(1,0,0), c2(0,1,0),st(0,0,0);
	computetan(centers[1], radius[1],centers[0], radius[0],startpoint);
	//cout << m << endl << "is unitary: " << m.isUnitary() << endl;
	//AngleAxis<float> aa(angle_in_radian, Vector3f(ax,ay,az));



	return 0;
	
}