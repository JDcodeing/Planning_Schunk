#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <limits>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
#include <moveit/planning_interface/planning_interface.h>


class mid_info
{
  public:
  Eigen::Vector3d pos;
  double dis;
  
  int index;

  mid_info():pos(Eigen::Vector3d(0,0,0)),dis(0.0),index(std::numeric_limits<int>::max()){};
  mid_info(Eigen::Vector3d p, double d, int ind):pos(p), dis(d),index(ind){};
  mid_info(Eigen::Vector3d p, double d):pos(p), dis(d),index(-1){};
  mid_info(const mid_info& rhs):pos(rhs.pos),dis(rhs.dis),index(rhs.index){};
  
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

	~FMGPlanner(){};

private:

	ros::NodeHandle node_handle_;
	ros::AsyncSpinner spinner;

	//robot_state::RobotState robot_state_;
	robot_model::RobotModelPtr kinematic_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
	const robot_state::JointModelGroup* joint_model_group_;

	std::vector<std::string> joint_names;


	// ros 
	ros::ServiceClient ik_service_client_;
	ros::Publisher traj_visualiser_;
	moveit_visual_tools::MoveItVisualToolsPtr visual_tools_, visual_tools_2;
  

	// for visualisation
	//moveit_msgs::DisplayTrajectory display_trajectory;


	// env bound 
	double bound = 1.0;

	//obstacles: 
	std::vector<Eigen::Vector3d> sphere_centers;
	std::vector<double> sphere_radius;
	std::vector<std::string> sphere_names;
	
	const double to_obs = 0.02;
	int obs_num=0;
	bool visualise_ = true;

	int pmids_num;
	std::vector<mid_info> pmids_vector; // midpoint set sorted
	std::vector<mid_info> Traj_mid_info;
	std::vector<Eigen::Vector3d> Traj_mid_pos;

	// improved 
	bool improve;

	// search graph
	std::vector<std::vector<bool> > connected ;  // 0:start; last: goal
	std::vector<std::vector<int> > searchnode; // the nodes connecting with the i_th midpoint 
	std::vector<int> searchnode_pointer_cur;
	std::vector<int> traj_mid_index; // initial trajectory expressed by the index of midpoint

	// Traj with joint values
	std::vector<std::vector<double> > interpTraj;
	std::vector<std::vector<double> > initpTraj;
	
	Eigen::Vector3d currentpos;
	Eigen::Vector3d goal;
	Eigen::Vector3d start;
	double cur2goal_dis;

	// to compute IK
	geometry_msgs::Pose pose_msg;

	Eigen::Affine3d start_state_affine;

	// dynamic 
	std::vector<mid_info> pmids_ObsEnv;
	int pmids_ObsEnv_size;
	double max_obs;
	std::vector<double> start_values;
	int IKsolve_forward;
	double smooth_tolerance;
	double max_cubic_stepsize;
	int only_look_Traj_mid_pos;
	double robotrange;

	 planning_interface::PlannerManagerPtr planner_instance;
	

public:
	bool plantraj(int nodenum, std::vector<int> traj, std::vector<int> searchnode_rec);
	bool replan_v2(int replan_startpoint, std::vector<int> oldtraj,std::vector<int> searchnode_rec);
	bool plan(float checkgoal);
	bool plan_dynamic(const Eigen::Vector3d start);
	bool get_dynamic_midpoints(const Eigen::Vector3d start, int recomputenum);
	bool get_dynamic_mid_pos(const Eigen::Vector3d start, int recomputenum);
	void test();
	void test_v2();
	void test_v2_collisionfree();
	void test_dynamic();
	void findCartesianPath();
	bool findCartesianPath_my(int recomputenum);
	void run();
	bool plan_cartesianpath_validpath(ros::Duration &time, double & len, bool display);
	int choice;

	void rrt_vs_fmg(int num);


private:
	int init();
	bool loadObs(std::string filename, bool display);
	void addObstoScene(); 
	void GenerateGaps();
	void GenerateGaps_dynamic(Eigen::Vector3d cur, std::vector<mid_info> &result);

	mid_info computetan(int index,const Eigen::Vector3d& c1, const double& r1, const Eigen::Vector3d& c2, const double& r2, const Eigen::Vector3d &cur);

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
		Traj_mid_info.push_back(midpoint);
	}
	bool checkSegment(const Eigen::Vector3d& con, const Eigen::Vector3d& point);
	bool checkPoseCollisionwithIK(const geometry_msgs::Pose & pose_msg, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs);
	bool checkCartPoseCollision(const Eigen::Vector3d &position, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs);
	bool checkCartPoseCollisionRand(const Eigen::Vector3d &position, double rand_range, bool randpos, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs);

	bool setconnet();
	bool Traj_interp_tomsgs();
	bool GetMidIKSolution();
	bool GetMidIKSolution(const std::vector<Eigen::Vector3d> &mid_points);
	void indextoTraj();

	// collision part
	bool checkState(const std::vector<double>& jv);
	bool regenerateIK(const Eigen::Affine3d endpose,  std::vector<double> &joint_values);
	bool modify_trajPoint(const int i);
	bool GetValidTraj();
	bool checkValidTraj(double fic, int& i);
	//bool checkValidTraj(double fic, std::vector<unsigned int> &invalid_index);
	bool Traj_validinterp_tomsgs(bool display , double &length);
	
	void print_waypoints(const std::vector<geometry_msgs::Pose> &waypoints);
	double checkSegment_dis2obs(const Eigen::Vector3d& con, const Eigen::Vector3d& point, Eigen::Vector3d & closepoint);
	void generategaps_ObsEnv(std::vector<mid_info>& result);
	void generategaps_CurEnv(const Eigen::Vector3d &cur, std::vector<mid_info>& result);
	bool smooth_traj();
	bool smooth_valid(const std::vector<double> &v1, const std::vector<double> &v2, double &max_diff);	
	bool AddCartesianPoint(const Eigen::Vector3d &pre, const Eigen::Vector3d &after,moveit_msgs::RobotState & moveit_init, moveit_msgs::RobotState &moveit_res);
	bool smoothBspline(std::vector<Eigen::Vector3d> &path, unsigned int maxSteps);
	void rangelimit(Eigen::Vector3d &position);

	bool benchmarkOMPL(ros::Duration &time, double & length, bool display);
	void toRosTrajectory(const std::vector<std::vector<double> >& points,
                      robot_trajectory::RobotTrajectory &rt);
	void omplsetup();


	//bool isMidFree(const Eigen::Vector3d& point);

};