#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <Eigen/Geometry>
#include "eigen_ros.hpp"
//#include <dr_eigen/yaml.hpp>
#include <moveit_msgs/GetPositionIK.h>
#include "spline.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <limits>

class mid_info
{
  public:
  Eigen::Vector3d pos;
  double dis;
  int index;
  mid_info():pos(Eigen::Vector3d(0,0,0)),dis(0.0),index(std::numeric_limits<int>::max()){};

  mid_info(Eigen::Vector3d p, double d, int ind):pos(p), dis(d),index(ind){};

  mid_info(Eigen::Vector3d p):pos(p), dis(0.0),index(-1){};
  
};
bool Mid_Greater( const mid_info& a, const mid_info& b)  { return a.dis > b.dis; }

class FMGPlanner
{
public:
	FMGPlanner():
		spinner(1),
		node_handle_("~")
		{

			spinner.start();
			pmids_vector.clear();
			Traj_mid_pos.clear();
		}

	~FMGPlanner();

private:

	ros::NodeHandle node_handle_;
	ros::AsyncSpinner spinner;

	robot_state::RobotState robot_state_;
	robot_model::RobotModelPtr kinematic_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	const robot_state::JointModelGroup* joint_model_group_;

	const std::vector<std::string> joint_names;


	// ros 
	ros::ServiceClient ik_service_client_;
	ros::Publisher traj_visualiser_;
	moveit_visual_tools::MoveItVisualTools visual_tools_;
  

	// for visualisation
	moveit_msgs::DisplayTrajectory display_trajectory;


	// env bound 
	double bound = 1.0;

	//obstacles: 
	std::vector<Eigen::Vector3d> sphere_centers;
	std::vector<double> sphere_radius;
	std::vector<std::string> sphere_names;
	
	const double to_obs = 0.02;
	int obs_num=3;
	bool visualise_ = true;

	int pmids_num;
	std::vector<mid_info> pmids_vector; // midpoint set sorted
	std::vector<mid_info> Traj_mid_pos;

	// improved 
	bool improve;

	// search graph
	std::vector<std::vector<bool> > connected ;  // 0:start; last: goal
	std::vector<vector<int> > searchnode; // the nodes connecting with the i_th midpoint 
	std::vector<int> searchnode_pointer_cur;
	std::vector<int> traj_mid_index; // initial trajectory expressed by the index of midpoint

	// Traj with joint values
	std::vector<std::vector<double> > interpTraj;
	std::vector<std::vector<double> > initpTraj;
	
	Eigen::Vector3d currentpos;
	Eigen::Vector3d goal;
	Eigen::Vector3d start;
	double cur2goal_dis;

	

public:
	bool plantraj(int nodenum, std::vector<int> traj, std::vector<int> searchnode_rec);
	bool replan_v2(int replan_startpoint, std::vector<int> oldtraj,std::vector<int> searchnode_rec);
	bool plan(float checkgoal);
	void test();
	void test_v2();
	int choice;

private:
	int init();
	bool loadObs(std::string filename, bool display);
	void addObstoScene(); 
	void GenerateGaps();
	bool isPointinObs(const Eigen::Vector3d& point)
	{
	  for(size_t i =0; i < obs_num; i++)
	  {
	      if((point - sphere_centers[i]).norm() < sphere_radius[i])
	        return true;
	  }
	  return false;
	}
	void AddMidPoint(const mid_info& midpoint)
	{
		std::cout<<"!!!!!!!!!!!!!!!!!!!add one mid!!" <<std::endl;
		Traj_mid_pos.push_back(point);
	}
	bool checkSegment(const Eigen::Vector3d& con, const Eigen::Vector3d& point);
	bool checkPoseCollision(const geometry_msgs::Pose & pose_msg, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs);
	bool checkCartPoseCollision(const Eigen::Vector3d &position, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs);

	bool setconnet();
	bool FMGPlanner::Traj_interp_tomsgs();
	bool Env2d::GetMidIKSolution();
	
	//bool isMidFree(const Eigen::Vector3d& point);

};