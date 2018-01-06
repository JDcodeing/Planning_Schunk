#include "FMGPlanner.h"
#include <random>
#include <chrono>
#include "util_fmg.h"
#include <moveit/move_group_interface/move_group.h>
#include <time.h>

int FMGPlanner::init()
{
	// load Params todo 
	try{

	ik_service_client_ = node_handle_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
	traj_visualiser_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/display_planned_path", 1, true);
  	visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_markers"));
  	//visual_tools_->loadTrajectoryPub("/moveit_manipulation/display_trajectory");
 	visual_tools_->loadMarkerPub();
  	visual_tools_->setAlpha(0.8);
  	visual_tools_->deleteAllMarkers();  // clear all old markers
  	visual_tools_->setManualSceneUpdating(true);
  	visual_tools_->loadRobotStatePub("/display_robot_state");
  	visual_tools_2.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_markers"));
  	//visual_tools_->loadTrajectoryPub("/moveit_manipulation/display_trajectory");
 	visual_tools_2->loadMarkerPub();
  	visual_tools_2->setAlpha(0.8);
  	visual_tools_2->deleteAllMarkers();  // clear all old markers
  	visual_tools_2->setManualSceneUpdating(true);
  	visual_tools_2->loadRobotStatePub("/display_robot_state2");
  	
  	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  	kinematic_model_ = robot_model_loader.getModel();
  	ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

  	//planning_scene_ = new planning_scene::PlanningScene(kinematic_model_);
  	//planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));
  	planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  	const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";
   	planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
   //	planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_monitor_->startSceneMonitor("/planning_scene");
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_ = planning_scene_monitor_->getPlanningScene();

  	robot_state::RobotState& robot_state_ = planning_scene_->getCurrentStateNonConst();
  	joint_model_group_ = robot_state_.getJointModelGroup("manipulator");
	joint_names = joint_model_group_->getVariableNames();

	//start state
	const Eigen::Affine3d &current_state = robot_state_.getGlobalLinkTransform("arm_6_link");
	start_state = current_state;
  	//ROS_INFO_STREAM("Translation: " << current_state.translation());
  	currentpos = current_state.translation();
  	start = currentpos;
  	cur2goal_dis = (currentpos-goal).norm();
  	robot_state_.copyJointGroupPositions(joint_model_group_, start_values);
  	initpTraj.clear();

  	pmids_ObsEnv_size = 0;
  	

  	//goal
  	node_handle_.getParam("goalx",goal(0));
  	node_handle_.getParam("goaly",goal(1));
  	node_handle_.getParam("goalz",goal(2));
  	node_handle_.getParam("planchoice", choice);
  	node_handle_.getParam("improve_plantraj", improve);
  	node_handle_.getParam("max_toobs", max_obs);
  	node_handle_.getParam("stepsize_cartesian", stepsize_cart);
  	node_handle_.getParam("IK_forward", IKsolve_forward);
  	node_handle_.getParam("smooth_tol", smooth_tolerance);
  	node_handle_.getParam("max_cubic_stepsize", max_cubic_stepsize);
  	node_handle_.getParam("bSpline_maxstep", bSpline_maxstep);
  	node_handle_.getParam("only_look_Traj_mid_pos",only_look_Traj_mid_pos);
  	

  	//random
  	srand (time(NULL));
  	pose_msg.orientation.x = fRand(-1,1);
   	pose_msg.orientation.y = fRand(-1,1);
    pose_msg.orientation.z = fRand(-1,1);
    pose_msg.orientation.w = fRand(-1,1);

  	


	return 0;

	}catch (const std::exception & e) {
		ROS_ERROR_STREAM(e.what());
		return 1;
	}
}
bool FMGPlanner::loadObs(std::string filename, bool displayObsInfo)
{
	std::ifstream obsfile(filename);
	int num = 0;
	if(obsfile.is_open())
	{
		std::string line;
		
		while(getline(obsfile, line))
		{
			if(line == "#sphere")
			{
				std::cout <<"***********!!!"<<std::endl;
				
				
				std::string name;
				obsfile >> name;
				double radious;
				obsfile >> radious;
				Eigen::Vector3d position;
				obsfile >> position(0) >> position(1) >> position(2);
				sphere_names.push_back(name);
				sphere_radius.push_back(radious);
				sphere_centers.push_back(position);
				std::cout <<name <<" "<<radious<<" "<<position<<std::endl;
				line = "";
				num++;
			}
			
		}
		obsfile.close();
		obs_num = num;

	if(displayObsInfo)
	{
		std::cout << "Load " << obs_num << " obstacles" <<std::endl;

		for(size_t i = 0; i < obs_num; i++)
		{
			std::cout << "Obstacle No. "<<i << ": radious: "<< sphere_radius[i] <<" center: " 
						<< sphere_centers[i][0]<<" "<< sphere_centers[i][1]<<" "<< sphere_centers[i][2] << std::endl;
		}
	}
		
		return true;

	}
	else
	{
		 std::cerr << "There was a problem opening the input file!\n";
		 return false;
	}
  
}
void FMGPlanner::addObstoScene()
{
	planning_scene_monitor_->getPlanningScene()->removeAllCollisionObjects();
	

	ros::Publisher planning_scene_diff_publisher = node_handle_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
	ros::ServiceClient planning_scene_diff_client =
	    node_handle_.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");
	    planning_scene_diff_client.waitForExistence();
  
  	//ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  	
  	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ROS_INFO("sleep");
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
    

    ros::Publisher pub_co = node_handle_.advertise<moveit_msgs::CollisionObject>("/collision_object", 100);
    
    shape_msgs::SolidPrimitive object;
    geometry_msgs::Pose pose;
    
    moveit_msgs::PlanningScene planning_scene;
    moveit_msgs::ApplyPlanningScene srv;
    
    ros::Rate rate(1);
    for(size_t i =0; i < obs_num; i++)
    {
    	moveit_msgs::CollisionObject co;
    	co.header.stamp = ros::Time::now();
	    co.header.frame_id = "world";
	    co.id = sphere_names[i];
	    co.operation = moveit_msgs::CollisionObject::ADD;

	    object.type = shape_msgs::SolidPrimitive::SPHERE;
	    object.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
  		object.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = sphere_radius[i];

	    //object.dimensions.push_back(sphere_radius[i]);
	    co.primitives.push_back(object);

	    
	    pose.position.x = sphere_centers[i](0);
	    pose.position.y = sphere_centers[i](1);
	    pose.position.z = sphere_centers[i](2);
	    pose.orientation.w = 1.0;
	    co.primitive_poses.push_back(pose);

	    pub_co.publish(co);
	    
	    ROS_INFO("Adding one object into the world");
	    
	    planning_scene.world.collision_objects.push_back(co);
	    
	    rate.sleep();
	    
	    
    }
    planning_scene.is_diff = true;
	    srv.request.scene = planning_scene;
	  	planning_scene_diff_client.call(srv);
	    planning_scene_diff_publisher.publish(planning_scene);
		ros::Duration(3).sleep(); 
		  	
}

void FMGPlanner::GenerateGaps(){}


void FMGPlanner::generategaps_ObsEnv(std::vector<mid_info>& result)
{
	if(pmids_ObsEnv_size<0)// checked already, the size is zero
	{
		ROS_ERROR_STREAM("checked already, the size is zero");
		return;
	}
	if(pmids_ObsEnv.size() > 0)
	{
		result.assign(pmids_ObsEnv.begin(), pmids_ObsEnv.end());
		ROS_INFO_STREAM("obsenv gaps size: " << result.size());
		return;
	}

	for(size_t i = 0; i < obs_num; i++)
	{
	  	
	  	// between the obs and the ceiling
	    double len = bound - sphere_centers[i](2);
	    Eigen::Vector3d pos;
	    double dis;
	    if (len > sphere_radius[i]) 
	    {
		    dis = (len-sphere_radius[i])*0.5;
		    pos = sphere_centers[i];
		    pos(2) = bound - dis;

		    if(!isPointinObs(pos))
		    {
		      pmids_ObsEnv.push_back(mid_info(pos,dis,pmids_ObsEnv_size));
		      pmids_ObsEnv_size++;
		    }
		}

	  	// between the obs and the floor
	    len = sphere_centers[i](2);
	    if (len > sphere_radius[i]) 
	    {
		    dis = (len-sphere_radius[i])*0.5;
		    pos = sphere_centers[i];
		    pos(2) = dis;
		    if(!isPointinObs(pos))
		    {
		      pmids_ObsEnv.push_back(mid_info(pos,dis,pmids_ObsEnv_size));
		      pmids_ObsEnv_size++;
		    }
		}

	    // between the obs and yz plane , front
	    len = bound - sphere_centers[i](0);
	    if (len > sphere_radius[i]) 
	    {
		    dis = (len-sphere_radius[i])*0.5;
		    pos = sphere_centers[i];
		    pos(0) = bound-dis;
		    if(!isPointinObs(pos))
		    {
		      pmids_ObsEnv.push_back(mid_info(pos,dis,pmids_ObsEnv_size));
		      pmids_ObsEnv_size++;
		    }
		}
	    // between the obs and yz plane , back
	    len = std::abs(- bound - sphere_centers[i](0));
	    if (len > sphere_radius[i]) 
	    {
		    dis = (len-sphere_radius[i])*0.5;
		    pos = sphere_centers[i];
		    pos(0) = -bound + dis;
		    if(!isPointinObs(pos))
		    {
		      pmids_ObsEnv.push_back(mid_info(pos,dis,pmids_ObsEnv_size));
		      pmids_ObsEnv_size++;
		    }
		}
	    // between the obs and xz plane , left
	    len = std::abs(- bound - sphere_centers[i](1));
	    if (len > sphere_radius[i]) 
	    {
		    dis = (len-sphere_radius[i])*0.5;
		    pos = sphere_centers[i];
		    pos(1) = -bound + dis;
		    if(!isPointinObs(pos))
		    {
		      pmids_ObsEnv.push_back(mid_info(pos,dis,pmids_ObsEnv_size));
		      pmids_ObsEnv_size++;
		    }
		}
	    // between the obs and xz plane , right
	    len = std::abs(bound - sphere_centers[i](1));
	    if (len > sphere_radius[i]) 
	    {
		    dis = (len-sphere_radius[i])*0.5;
		    pos = sphere_centers[i];
		    pos(1) = bound - dis;
		    if(!isPointinObs(pos))
		    {
		      pmids_ObsEnv.push_back(mid_info(pos,dis,pmids_ObsEnv_size));
		      pmids_ObsEnv_size++;
		    }
		}
		
		
	}
	if(pmids_ObsEnv_size==0)
		pmids_ObsEnv_size = -1;
	else
	{
		result.assign(pmids_ObsEnv.begin(), pmids_ObsEnv.end());
	}
	return;

}

// not used
void FMGPlanner::generategaps_CurEnv(const Eigen::Vector3d &cur, std::vector<mid_info>& result)
{
	int index = result.size();
		double ee_radius = 0.1;
	    double len = bound - cur[2];
	    double dis;
	    Eigen::Vector3d pos;
	    // between the cur and the ceiling
	    if (len > ee_radius) 
		{
			dis = (len-ee_radius)*0.5;
		    pos = cur;
		    pos(2) = bound - dis;
		    if(!isPointinObs(pos))
		    {
		      result.push_back(mid_info(pos,0,index++));
		      
		    }
		}   

	  	// between the obs and the floor
	    len = cur(2);
	    if (len > ee_radius) 
		{
			dis = (len-ee_radius)*0.5;
		    pos = cur;
		    pos(2) = dis;
		    if(!isPointinObs(pos))
		    {
		      result.push_back(mid_info(pos,0,index++));
		      
		    }
		}  


	    // between the obs and yz plane , front
	    len = bound - cur(0);
	    if (len > ee_radius) 
		{
			dis = (len-ee_radius)*0.5;
		    pos = cur;
		    pos(0) = bound - dis;
		    if(!isPointinObs(pos))
		    {
		      result.push_back(mid_info(pos,0,index++));
		      
		    }
		}  
	    // between the obs and yz plane , back
	    len = std::abs(- bound - cur(0));
	    if (len > ee_radius) 
		{
			dis = (len-ee_radius)*0.5;
		    pos = cur;
		    pos(0) = - bound + dis;
		    if(!isPointinObs(pos))
		    {
		      result.push_back(mid_info(pos,0,index++));
		      
		    }
		}  
	    // between the obs and xz plane , left
	    len = std::abs(- bound - cur(1));
	    if (len > ee_radius) 
		{
			dis = (len-ee_radius)*0.5;
		    pos = cur;
		    pos(1) = -bound + dis;
		    if(!isPointinObs(pos))
		    {
		      result.push_back(mid_info(pos,0,index++));
		      
		    }
		}  
	    // between the obs and xz plane , right
	    len = std::abs(bound - cur(1));
	    if (len > ee_radius) 
		{
			dis = (len-ee_radius)*0.5;
		    pos = cur;
		    pos(1) = bound - dis;
		    if(!isPointinObs(pos))
		    {
		      result.push_back(mid_info(pos,0,index++));
		      
		    }
		}  
		
	return;
}
void FMGPlanner::GenerateGaps_dynamic(Eigen::Vector3d cur, std::vector<mid_info> &result)
{
	ROS_INFO_STREAM("before start gaps size: " << result.size());
	generategaps_ObsEnv(result);
	for(int i = 0; i < result.size(); i++)
	{
		ROS_INFO_STREAM(result[i].pos);
	}
	
	int index = result.size();
	ROS_INFO_STREAM("initial gaps size: " << index);
	for(size_t i = 0; i < obs_num-1; i++)
	{
		for(size_t j = i+1; j < obs_num; j++)
		{
			mid_info gap = computetan(index, sphere_centers[i], sphere_radius[i]+to_obs,
									sphere_centers[j], sphere_radius[j]+to_obs, cur);

			result.push_back(gap);
			index++;
		}
	}
	generategaps_CurEnv(cur, result);
	std::sort(result.begin(), result.end(),Mid_Greater);
	result.insert(result.begin(), mid_info(goal,0,-1));

}

mid_info FMGPlanner::computetan(int index,const Eigen::Vector3d& c1, const double& r1, const Eigen::Vector3d& c2, const double& r2, const Eigen::Vector3d &cur)
{
	Eigen::Vector3d cur2c1_dir = (c1-cur).normalized();
	Eigen::Vector3d cur2c2_dir = (c2-cur).normalized();
	Eigen::Vector3d axisz = cur2c1_dir.cross(cur2c2_dir);


	//the tan point on c1
	Eigen::Vector3d c1_yaxis = axisz.cross(cur2c1_dir);
	
	double c1_angle = asin(r1/((c1-cur).norm()));
	double c1len = sqrt((c1-cur).dot(c1-cur) - r1*r1);
	Eigen::Matrix3d m;
	//cout << cur2c1_dir << endl;
	//cout << endl<< axisz << endl;
	m = Eigen::AngleAxisd(c1_angle, axisz.normalized());
	Eigen::Vector3d c1tan_framec1 = m*cur2c1_dir*c1len;
	Eigen::Vector3d c1tan = c1tan_framec1 + cur;
	
	//the tan point on c2
	double c2_angle = asin(r2/((c2-cur).norm()));
	double c2len = sqrt((c2-cur).dot(c2-cur) - r2*r2);
	m = Eigen::AngleAxisd(-c2_angle, axisz.normalized());
	Eigen::Vector3d c2tan_framec1 = m*cur2c2_dir*c2len;
	Eigen::Vector3d c2tan = c2tan_framec1 + cur;

	// checking if the mid point is free
	Eigen::Vector3d mid = (c1tan+c2tan)*0.5;
	double mid_dis = (c1tan-c2tan).norm();
	return mid_info(mid,mid_dis,index);
}


double FMGPlanner::checkSegment_dis2obs(const Eigen::Vector3d& con, const Eigen::Vector3d& point, Eigen::Vector3d &closestpoint)
{
  	//std::cout<<"&&&&&&&&&&&&&&&&&&& checking is free point:"<<point<<std::endl;
	//if(point.norm()>0.8)
	//{
	//	ROS_INFO_STREAM(point);
	//	return -1; // cannot reach the point
	//}
	double dis_obs_min = std::numeric_limits<double>::max();
	double dis_cu2p = (point - con).norm();
  
  	if(dis_cu2p < 0.02)
    	return -1; // too near
  	Eigen::Vector3d dir_cu2p = (point - con).normalized();
  	//std::cout<<"&&&&&&&&&&&&&&&&&&& dir_cu2p:"<<dir_cu2p<<std::endl;

  	for(size_t i =0; i < obs_num; i++)
  	{
    	if((point - sphere_centers[i]).norm() < sphere_radius[i])
        	return -1;

      	Eigen::Vector3d cur2obs = sphere_centers[i] - con;
    	//  std::cout<<"&&&&&&&&&&&&&&&&&&& cur2obs:"<<cur2obs<<std::endl;
      	double proj = cur2obs.dot(dir_cu2p);
    	//  std::cout<<"&&&&&&&&&&&&&&&&&&& dis_cup:"<<dis_cu2p<<" proj: "<<proj<<std::endl;
      	if(proj<0)
      	{
      		double dis = cur2obs.norm() - sphere_radius[i];
      		if(dis_obs_min > dis) dis_obs_min = dis;
      		continue;
      	} 
      	if(proj>dis_cu2p) 
      	{
      		double dis = (sphere_centers[i] - point).norm() - sphere_radius[i];
      		if(dis_obs_min > dis) dis_obs_min = dis;
      		continue;   
      	}

	    //  Eigen::Vector3d closestpoint = proj*dir_cu2p+con;
    	//  std::cout << "closest point " << closestpoint << std::endl;
      	double closest_dis = sqrt(cur2obs.dot(cur2obs) - proj*proj)-sphere_radius[i];
     	 //double closest_dis2 = (sphere_centers[i] - closestpoint).norm() - sphere_radius[i];
    	//  std::cout<<"&&&&&&&&&&&&&&&&&&& closest_dis:"<<closest_dis<<" "<<closest_dis2<<std::endl;
      	if(closest_dis  > 0)
      	{
      		if(closest_dis < dis_obs_min) 
      		{
      			dis_obs_min = closest_dis;
      			Eigen::Vector3d obs2closp = proj*dir_cu2p - cur2obs;
      			closestpoint = (max_obs+sphere_radius[i])*(obs2closp.normalized()) + sphere_centers[i];
      		}
      	}
      	else
      	{
      		return -1;
      	}
  	}
	//  std::cout << point(0)<<" " << point(1) << " " << point(2) << "is free!!";
  	if(dis_obs_min<std::numeric_limits<double>::max())
  	{
  		//ROS_INFO_STREAM("check point "<< point);
  		return dis_obs_min;
  	}
  	//return dis_obs_min;
	
	return -1;
}

bool FMGPlanner::checkSegment(const Eigen::Vector3d& con, const Eigen::Vector3d& point)
{return true;}

bool FMGPlanner::checkPoseCollisionwithIK(const geometry_msgs::Pose & pose_msg, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs)
{
	moveit_msgs::GetPositionIK::Request ik_req;
	ik_req.ik_request.group_name = "manipulator";
	ik_req.ik_request.avoid_collisions = true;

	geometry_msgs::PoseStamped pose_s;
    pose_s.header.stamp = ros::Time::now();
    pose_s.header.frame_id = "world";
    pose_s.pose = pose_msg;

    ik_req.ik_request.robot_state = init_state_msgs;
    ik_req.ik_request.pose_stamped = pose_s;
    ik_req.ik_request.timeout = ros::Duration(0.01);
  //  ik_req.ik_request.attempts = 5;

    moveit_msgs::GetPositionIK::Response ik_res;

    if(ik_service_client_.call(ik_req, ik_res))
    {
    	if (ik_res.error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION){
        ROS_INFO("*******************no ik solution!!!!**************");
        return false;
        } 
        else
        {
        	res_state_msgs = ik_res.solution;
        	init_state_msgs = res_state_msgs;
        	return true;
        }
    }
    else
    {
    	return false;
    }
      
}

bool FMGPlanner::smooth_valid(const std::vector<double> &v1, const std::vector<double> &v2, double &max_diff)
{
	
	if(v1.size() != v2.size())
	{
		ROS_ERROR_STREAM("smooth_valid function : size not match!");
		return false;
	}
	for(int i=0; i < v1.size()-1; i++)
	{
		double v = std::abs(v1[i] - v2[i]);
		if(max_diff < v) max_diff = v;
	}
	std::cout<<"the max is: " << max_diff<<" ";
	if(max_diff < smooth_tolerance) return true;
	/*else {
		for(int i = 0; i < v1.size();i++)
		{
			std::cout << v1[i]<< "    "<<v2[i]<<std::endl;

		}*/
	return false;
	//}
}

bool FMGPlanner::smooth_traj()
{
	
	std::vector<double> last_values(initpTraj.back());
	int initp_size = initpTraj.size();
	ROS_INFO_STREAM("smooth traj begin! the size is "<< initp_size);
	double max_diff;
	int invalidnum=0;
	for(int i = initp_size-2; i >= 0; i--)
	{	
		std::cout << "the "<< i <<" th smooth check: last values"<<std::endl;
		for(int k = 0; k < last_values.size();k++)
		{
			std::cout << last_values[k]<< "    ";

		}
		double maxdiff_tmp;
		if(smooth_valid(initpTraj[i], last_values,maxdiff_tmp))
		{
			last_values.assign(initpTraj[i].begin(),initpTraj[i].end());
			continue;
		}
		else
		{
			
			bool recomputevalid = false;
			std::vector<double> joint_values;

			
				moveit_msgs::RobotState moveit_init;
				moveit_msgs::RobotState moveit_res;
				
				robot_state::RobotState& robot_state_ = planning_scene_->getCurrentStateNonConst();;
				
	  			robot_state_.setJointGroupPositions(joint_model_group_, last_values);
			  	moveit::core::robotStateToRobotStateMsg(robot_state_,moveit_init);

			  	robot_state_.setJointGroupPositions(joint_model_group_, initpTraj[i]);
	  			const Eigen::Affine3d &ee_pose = robot_state_.getGlobalLinkTransform("arm_6_link");
			  
				unsigned int trynum = 0;
				while(!recomputevalid && (trynum++ < 5))
				{
					if(! checkCartPoseCollisionRand(ee_pose.translation(),0.05,trynum,moveit_init,moveit_res))
					{
						recomputevalid = false;
						continue;
					}
					moveit::core::robotStateMsgToRobotState(moveit_res, robot_state_);
			        robot_state_.copyJointGroupPositions(joint_model_group_, joint_values);

			        if(smooth_valid(joint_values, last_values,maxdiff_tmp))
			        {
			        	recomputevalid = true;
						std::cout << "the" <<i<<" the recompute display one state!!"<<std::endl;
						visual_tools_->publishRobotState(robot_state_, rviz_visual_tools::GREEN);
						initpTraj[i] = joint_values;
						last_values.assign(joint_values.begin(),joint_values.end());
						ros::Duration(3).sleep();	
						break;	
			        }
			        else // not smooth_valid
			        {
			        	recomputevalid = false;
			        	if(maxdiff_tmp < max_diff)
			        	{
			        		initpTraj[i] = joint_values;
							last_values.assign(joint_values.begin(),joint_values.end());
			        	}

			        }
				}
				if(!recomputevalid)
				{
					invalidnum++;
					ROS_ERROR_STREAM("found one unsmooth state: " <<i<<"th state");
				}			
				
		}
	}
	if(invalidnum) return false;
	return true;
}

bool FMGPlanner::regenerateIK(const Eigen::Affine3d endpose,  std::vector<double> &joint_values)
{return true;}

bool FMGPlanner::checkCartPoseCollisionRand(const Eigen::Vector3d &position, double rand_range, bool randpos, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs)
{
	Eigen::Vector3d randposition = position;
	if(randpos)
	{
		randposition = Vector3dRand(position,-rand_range,rand_range);
	}
  	pose_msg.position = fmg::toRosPoint(randposition);
  	return (checkPoseCollisionwithIK(pose_msg,init_state_msgs,res_state_msgs));
}

bool FMGPlanner::checkCartPoseCollision(const Eigen::Vector3d &position, moveit_msgs::RobotState &init_state_msgs, moveit_msgs::RobotState &res_state_msgs)
{
  	pose_msg.position = fmg::toRosPoint(position);
  	return (checkPoseCollisionwithIK(pose_msg,init_state_msgs,res_state_msgs));
}



bool FMGPlanner::plan(float checkgoal)
{
	return true;
}

bool FMGPlanner::setconnet()
{
	return true;
}
bool FMGPlanner::plantraj(int nodenum, std::vector<int> traj, std::vector<int> searchnode_rec)
{
	return true;
}

bool FMGPlanner::replan_v2(int replan_startpoint, std::vector<int> oldtraj,std::vector<int> searchnode_rec)
{
	return true;
}

bool FMGPlanner::get_dynamic_midpoints(const Eigen::Vector3d start, int recomputenum)
{
	return true;	
}

bool FMGPlanner::get_dynamic_mid_pos(const Eigen::Vector3d start, int recomputenum)
{
	Eigen::Vector3d cur = start;
	std::vector<mid_info> GapSet;
	 
	int stepnum = 0;
  	Traj_mid_pos.push_back(cur);
  	while(cur!=goal)
	{
		
		GapSet.clear();
		GenerateGaps_dynamic(cur, GapSet);

		if(GapSet.size()<1)
		{
			std::cout << "plan fail!! Gap set is empty!!" <<std::endl;
			return false;
		}
		bool foundone = false;

		Eigen::Vector3d closestpoint;
		
		for(int j = 0; j < GapSet.size(); j++)
		{
			const mid_info& onemid = GapSet[j];
			Eigen::Vector3d position = onemid.pos;
			// check if near
			if((position - goal).norm() < (cur - goal).norm())
			{
				// check if free and the shortest distance to obstacles
				double segvalid = checkSegment_dis2obs(cur, position,closestpoint);
				if(segvalid<0) // segvalid < 0 means it collides with obs
				{continue;}
				
				else if(segvalid >= max_obs) // find a valid midpoint
				{
  
					if(stepnum==0 && recomputenum>0) // when recomputing, find next valid midpoint
					{
						recomputenum--;
						continue;
					}
					else
					{
						if(position.norm()>robotrange)
						{
							position = robotrange*(position.normalized());
						}
						Traj_mid_pos.push_back(position);
						cur = position;
						foundone = true;
						std::cout << "!!!!!!!!!!!!!!!one step !!!!!!!:  " << stepnum << std::endl;
						ROS_INFO_STREAM("the segvalid is: " << segvalid);
						std::cout << onemid.pos << std::endl;
						break;
						
					}
					
				}
				else
				{

					//check next mid point
					Eigen::Vector3d tmp;
					if(checkSegment_dis2obs(cur,closestpoint,tmp) && checkSegment_dis2obs(closestpoint,onemid.pos,tmp))
					{
						Traj_mid_pos.push_back(closestpoint);
						Traj_mid_pos.push_back(onemid.pos);
						cur = onemid.pos;
						foundone = true;
						ROS_INFO_STREAM("segment too close to obs: " << segvalid<<"but add one more");
						break;
					}
					else
					{
						ROS_INFO_STREAM("segment too close to obs: " << segvalid<<" and failed to add one more");
					}
					
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

	return true;	
}


bool FMGPlanner:: plan_dynamic(const Eigen::Vector3d start)
{
	return true;
}


void FMGPlanner::indextoTraj()
{}

bool FMGPlanner::checkState(const std::vector<double>& jv)
{
	
    robot_state::RobotState& current_state_r = planning_scene_->getCurrentStateNonConst();
    
    std::vector<double> jv_n;
    
    /*
    current_state_r.copyJointGroupPositions(joint_model_group_, joint_values);
    joint_values[0] = values[0];
    joint_values[1] = values[1];
    joint_values[2] = values[2];
    joint_values[3] = values[3];
    joint_values[4] = values[4];
    joint_values[5] = values[5];
    */
    //ROS_INFO("CheckState!! Jv = [ %f, %f, %f, %f, %f, %f ]", jv[0], jv[1], jv[2], jv[3], jv[4], jv[5] );
    current_state_r.setJointGroupPositions(joint_model_group_, jv);
    current_state_r.update();
   // current_state_r.copyJointGroupPositions(joint_model_group_, jv_n);
    
    bool valid = planning_scene_monitor_->getPlanningScene()->isStateValid(current_state_r, "manipulator");
   // std::cout<< ha << "*************"<<std::endl;
    

    /*if(!valid)
    {
    	ROS_INFO("invalide!! Jv = [ %f, %f, %f, %f, %f, %f ]", jv[0], jv[1], jv[2], jv[3], jv[4], jv[5] );
    	visual_tools_->publishRobotState(current_state_r, rviz_visual_tools::RED);
	
		ros::Duration(5).sleep();
    }*/
    return valid;
    

}

bool FMGPlanner::modify_trajPoint(const int invalid_index)
{
	return true;
}

bool FMGPlanner::GetValidTraj()
{
	return true;
}

bool FMGPlanner::checkValidTraj(double fic, int &invalid_index)
{
	bool has = planning_scene_monitor_->getPlanningScene()->getCollisionWorld()->getWorld()->hasObject("obs1");
    std::cout << "obs1: " << has;
    has = planning_scene_monitor_->getPlanningScene()->getCollisionWorld()->getWorld()->hasObject("obs2");
    std::cout << "obs2: " << has;
    has = planning_scene_monitor_->getPlanningScene()->getCollisionWorld()->getWorld()->hasObject("obs3");
    std::cout << "obs2: " << has;

	int TrajSize = interpTraj.size();

	int step = TrajSize*fic;
	//step = 1;
	std::cout << "TrajSize: "<< TrajSize<<" and step: "<<step << std::endl;
	//robot_state::RobotState& kinematic_state = planning_scene_->getCurrentStateNonConst();
	for(int i = 0; i < TrajSize; i=i+step)
	{
		std::vector<double> jv = interpTraj[i];
		//vector<double> newIKsol(6,0);
		std::cout << "i th: "<<i <<" ";
		for(auto j: jv) std::cout<<j <<" ";
			std::cout << std::endl;
		if(!checkState(jv))
		{
			invalid_index = i;
			return false;
		}
	}
	return true;
}


bool FMGPlanner::Traj_interp_tomsgs()
{
  	return true;
}

bool FMGPlanner::Traj_validinterp_tomsgs()
{
	
	moveit_msgs::RobotTrajectory joint_solution = fmgplanner::toROSJointTrajectory(interpTraj, joint_names, 1.0);
  	ROS_INFO("Visualizing the trajectory");
  	display_trajectory.trajectory.push_back(joint_solution);
  	traj_visualiser_.publish(display_trajectory);
  	return true;
  
}
bool FMGPlanner::AddCartesianPoint(const Eigen::Vector3d &pre, const Eigen::Vector3d &after,moveit_msgs::RobotState & moveit_init, moveit_msgs::RobotState &moveit_res)
{
	Eigen::Vector3d mid,tmp;
	fmgplanner::interpolate(pre,after,0.5, mid);
	if(!checkSegment_dis2obs(pre,mid,tmp) || !checkSegment_dis2obs(mid,after,tmp))
	{
		ROS_ERROR_STREAM("AddCartesianPoint Segment Check failed!");
		return false;
	}
	
	bool PoseValid = checkCartPoseCollisionRand(mid,0.05,false,moveit_init,moveit_res);
	unsigned int trynum = 3;
	while(!PoseValid && trynum--)
	{
		PoseValid = checkCartPoseCollisionRand(mid,0.05,true,moveit_init,moveit_res);
	}
	return PoseValid;

}
bool FMGPlanner::GetMidIKSolution(const std::vector<Eigen::Vector3d> &mid_points)
{
	size_t mid_size = mid_points.size();
	if(mid_size <1)
	{
		ROS_ERROR_STREAM("Mid Traj Size < 2 !!");
		return false;
	}
	std::vector<double> joint_values;
	moveit_msgs::RobotState moveit_init;
	moveit_msgs::RobotState moveit_res;
	//std::cout <<" the init size:: "<< initpTraj.size()<<std::endl;
	if(initpTraj.size()>1)
	{
		initpTraj.clear();
	}
	robot_state::RobotState& robot_state_ = planning_scene_->getCurrentStateNonConst();
	moveit::core::robotStateToRobotStateMsg(robot_state_,moveit_init);
  	bool PoseValid = 0;
  	int invalidnum = 0;
  	IKsolve_forward = 1;
  	if(IKsolve_forward==1)
  	{
  		initpTraj.push_back(start_values);
		for(size_t i = 1; i < mid_size; i++)
		{
			if(!PoseValid && i>1)
			{
				bool addone = AddCartesianPoint(mid_points[i-1],mid_points[i],moveit_init,moveit_res);
				if(addone)
				{
					moveit::core::robotStateMsgToRobotState(moveit_res, robot_state_);
	           		robot_state_.copyJointGroupPositions(joint_model_group_, joint_values);
	           		std::cout << "forward display one state!!"<<std::endl;
					visual_tools_->publishRobotState(robot_state_, rviz_visual_tools::WHITE);
					moveit_init = moveit_res;
					initpTraj.push_back(joint_values);
					ros::Duration(3).sleep();
				}
			}

			PoseValid = checkCartPoseCollisionRand(mid_points[i],0.05,false,moveit_init,moveit_res);
			unsigned int trynum = 3;
			while(!PoseValid && trynum--)
			{
				PoseValid = checkCartPoseCollisionRand(mid_points[i],0.05,true,moveit_init,moveit_res);
			}
			if(PoseValid)
			{
				moveit::core::robotStateMsgToRobotState(moveit_res, robot_state_);
	           	robot_state_.copyJointGroupPositions(joint_model_group_, joint_values);
	           	std::cout << "forward display one state!!"<<std::endl;
				visual_tools_->publishRobotState(robot_state_, rviz_visual_tools::GREEN);

				moveit_init = moveit_res;
				initpTraj.push_back(joint_values);
				ros::Duration(3).sleep();
			}
			else
				{
					std::cout << "the "<< i <<" th mid point IK failed!!"<<std::endl;
					std::cout << mid_points[i]<<std::endl;
					invalidnum++;
					continue;
				}

		}  		
  	}
  	
	
	if(invalidnum>0) return false;
	return true;

}

bool FMGPlanner::GetMidIKSolution()
{
	return true;
}

void FMGPlanner::test()
{}

void FMGPlanner::test_v2()
{} 

void FMGPlanner::test_v2_collisionfree()
{} 


void FMGPlanner::test_dynamic()
{}

void FMGPlanner::print_waypoints(const std::vector<geometry_msgs::Pose> &waypoints)
{
	ROS_INFO_STREAM("-------print waypoints!!!!----");
	int num = waypoints.size();
	if(num>0)
	{
		for(int i = 0; i < num; i++ )
			ROS_INFO_STREAM(waypoints[i]);
	}
}
void FMGPlanner::findCartesianPath()
{}



bool FMGPlanner::smoothBspline(std::vector<Eigen::Vector3d> &path, unsigned int maxSteps)
{
	if (path.size() < 3)
        return false;

    bool suc = false;

    Eigen::Vector3d temp1,temp2,tmp;
    for (unsigned int s = 0; s < maxSteps; ++s)
    {
        //
        if(s>0) fmgplanner::subdivide(path);

        unsigned int i = s>0?2:1, u = 0, n1 = path.size() - 1;
        while (i < n1)
        {
            //if (si->isValid(states[i - 1]))
            //{
                fmgplanner::interpolate(path[i - 1], path[i], 0.5, temp1);
                fmgplanner::interpolate(path[i], path[i + 1], 0.5, temp2);
                fmgplanner::interpolate(temp1, temp2, 0.5, temp1);
                if (checkSegment_dis2obs(path[i - 1], temp1,tmp) && checkSegment_dis2obs(temp1, path[i + 1],tmp))
                {
                	path[i] = temp1;
                	++u;
                }
            //}

            i += 2;
        }

        if (u == 0)
            break;
        suc = true;
        
    }
    return true;

}

bool FMGPlanner::findCartesianPath_my(int recomputenum)
{
	std::cout << "test_dynamic : the start is"<<std::endl;
	std::cout << start<<std::endl;
	if(Traj_mid_pos.size()>0) Traj_mid_pos.clear();

	// find midpoints from start to goal dynamicly
	if(! get_dynamic_mid_pos(start,recomputenum)) //get Traj_mid_pos
	{
		ROS_ERROR_STREAM("get Traj_mid_pos failed!");
		return false;
	}
	else
	{
		ROS_INFO_STREAM("the Traj_mid_pos: , the "<<recomputenum<<" th computation");
		for(int i = 0; i < Traj_mid_pos; i++)
			ROS_INFO(Traj_mid_pos[i]);
	}
	if(only_look_Traj_mid_pos) return false;

	ROS_INFO_STREAM("smoothBspline! size: "<< Traj_mid_pos.size());
	if(smoothBspline(Traj_mid_pos,bSpline_maxstep))
	{
		ROS_INFO_STREAM("smoothBspline succeed! size: "<< Traj_mid_pos.size());
	}
	ROS_INFO_STREAM("The initial Cartesian Path, state size: "<< Traj_mid_pos.size());

	// interpolate the Cartesian path 
	fmgplanner::interpolateCartesianPath(Traj_mid_pos, stepsize_cart);
	
	ROS_INFO_STREAM("The interpolated Cartesian Path, state size: "<< Traj_mid_pos.size());

	// compute IK solutions of all midpoints
	bool init_IKsolved = GetMidIKSolution(Traj_mid_pos);
	if(init_IKsolved)
		ROS_INFO_STREAM("Initial path : all IK solutions found!");
	else
		ROS_ERROR_STREAM("Initial path : not all IK solution found ");

	// if the difference between two sets of consecutive joint values is larger than "smooth_tolerance", recompute IK solution
	// the process is done backward
	// output : initpTraj
	bool smooth = smooth_traj();
	if(smooth) 
		ROS_INFO_STREAM("Smoothing Completed!");
	else
		ROS_ERROR_STREAM("Smoothing failed!");

	fmgplanner::printtraj(initpTraj);

	//return false;
	// get the trajectory via cubic interpolation
	bool interp_suc = fmgplanner::cubic_interp(interpTraj, initpTraj,max_cubic_stepsize);
	if(!interp_suc)
	{
		ROS_ERROR_STREAM("Cubic interpolation failed!");
		return false;
	}

	// check if the trajectory is valid ; stepsize = fic * interpTraj.size()
	int invalidind;
	bool traj_valid = checkValidTraj(0.05, invalidind);
	if(traj_valid)
	{
		ROS_INFO_STREAM("Valid Trajectory found!");
	}
	else
	{
		ROS_ERROR_STREAM("not found valid trajectory");
		return false;
	}
	return true;

}

void FMGPlanner::plan_cartesianpath_validpath()
{
	int trynum = 5;
	for(double i = 0; i < trynum; i=i+1)
	{
		if(findCartesianPath_my((int)floor(i)))
		{
			Traj_validinterp_tomsgs();
			break;
		}
	}
}

void FMGPlanner::run()
{
	init();
	loadObs("/home/azalea-linux/ws_moveit/src/planning_test/src/obs.scene",true);
	addObstoScene();
	//return;	//GenerateGaps();
	std::cout<<" the planning choice is " << choice << std::endl;
	if(this->choice==0)
	{
		this->test();
	}
	else if(this->choice==1)
	{
		this->test_v2();
	}
	else if(this->choice==2)
	{
		this->test_v2_collisionfree();
	}
	else if(this->choice == 3)
	{
		this->findCartesianPath();
	}
	else if(this->choice == 4)
	{
		this->plan_cartesianpath_validpath();
	}
	else if(this->choice==5){
		this->test_dynamic();
	}
	else
	{
		return;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fmgplanner");
	FMGPlanner planner;
	

	planner.run();

	ros::shutdown();

  	return 0;
}


