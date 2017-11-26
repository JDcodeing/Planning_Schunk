#include "FMGPlanner.h"
#include <random>
#include <chrono>
#include "util_fmg.h"

int FMGPlanner::init()
{
	// load Params todo 
	try{

	ik_service_client_ = node_handle_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
	traj_visualiser_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  	visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_markers");
  	//visual_tools_->loadTrajectoryPub("/moveit_manipulation/display_trajectory");
 	visual_tools_->loadMarkerPub();
  	visual_tools_->setAlpha(0.8);
  	visual_tools_->deleteAllMarkers();  // clear all old markers
  	visual_tools_->setManualSceneUpdating(true);
  	visual_tools_->loadRobotStatePub("/fgm_planner/display_robot_state");
  	
  	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  	kinematic_model_ = robot_model_loader.getModel();
  	ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

  	//planning_scene_ = new planning_scene::PlanningScene(kinematic_model_);
  	planning_scene_ = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(kinematic_model_));
  	robot_state_ = planning_scene_->getCurrentStateNonConst();
  	joint_model_group_ = robot_state_.getJointModelGroup("manipulator");
	joint_names = joint_model_group_->getVariableNames();

	//start state
	const Eigen::Affine3d &current_state = robot_state_.getGlobalLinkTransform("arm_6_link");
  	//ROS_INFO_STREAM("Translation: " << current_state.translation());
  	currentpos = current_state.translation();
  	cur2goal_dis = (currentpos-goal).norm();
  	std::vector<double> joint_values;
  	robot_state_.copyJointGroupPositions(joint_model_group_, joint_values);
  	initpTraj.clear();
  	initpTraj.push_back(joint_values);

  	//goal
  	node_handle_.getParam("goalx",goal(0));
  	node_handle_.getParam("goaly",goal(1));
  	node_handle_.getParam("goalz",goal(2));
  	node_handle_.getParam("planchoice", choice);
  	


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
				
				
				std::string name;
				obsfile >> name;
				double radious;
				obsfile >> radious;
				Eigen::Vector3d position;
				obsfile >> position(0) >> position(1) >> position(2);
				sphere_names.push_back(name);
				sphere_radius.push_back(radious);
				sphere_centers.push_back(position);
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
						<< sphere_centers[i](0)<<" "<< sphere_centers[i](1)<<<" "< sphere_centers[i](2) << std::endl;
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
	//ros::Publisher planning_scene_diff_publisher = node_handle_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	ros::ServiceClient planning_scene_diff_client =
	    node_handle_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
	    planning_scene_diff_client.waitForExistence();
  
  
  	/*
  	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ROS_INFO("sleep");
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
    */

    //ros::Publisher pub_co = node_handle_.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    moveit_msgs::CollisionObject co;
    shape_msgs::SolidPrimitive object;
    geometry_msgs::Pose pose;
    
    moveit_msgs::PlanningScene planning_scene;
    moveit_msgs::ApplyPlanningScene srv;
    for(size_t i =0; i < obs_num; i++)
    {
    	co.header.stamp = ros::Time::now();
	    co.header.frame_id = "world";
	    co.id = sphere_names[i];
	    co.operation = moveit_msgs::CollisionObject::ADD;

	    object.type = shape_msgs::SolidPrimitive::SPHERE;
	    object.dimensions.push_back(sphere_radius[i]);
	    co.primitives.push_back(object);

	    
	    pose.position.x = sphere_centers[x](0);
	    pose.position.y = sphere_centers[x](1);
	    pose.position.z = sphere_centers[x](2);
	    pose.orientation.w = 1.0;
	    co.primitive_poses.push_back(pose);

	    //pub_co.publish(co);
	    
	    ROS_INFO("Adding one object into the world");
	    
	    planning_scene.world.collision_objects.push_back(co);
	    planning_scene.is_diff = true;
	    srv.request.scene = planning_scene;
	  	planning_scene_diff_client.call(srv);
	    //planning_scene_diff_publisher.publish(planning_scene);
	    
    }
 	  	
}

void FMGPlanner::GenerateGaps()
{
	if(obs_num <1)
	{
		ROS_INFO("No obstacles")
	}
	  double dis;
	  Eigen::Vector3d pos;
	  for(size_t i = 0; i < obs_num; i++)
	  {

	  	// between the obs and the floor
	    double len = sphere_centers[i](2);
	    if (len <= sphere_radius[i]) continue;
	    dis = (len-sphere_radius[i])*0.5;
	    pos = sphere_centers[i];
	    pos(2) = dis;
	    if(!isPointinObs(pos))
	    {
	      pmids_vector.push_back(mid_info(pos,dis));
	    }

	    // between the obs and yz plane , front
	    len = bould - sphere_centers[i](0);
	    if (len <= sphere_radius[i]) continue;
	    dis = (len-sphere_radius[i])*0.5;
	    pos = sphere_centers[i];
	    pos(0) = bound - dis;
	    if(!isPointinObs(pos))
	    {
	      pmids_vector.push_back(mid_info(pos,dis));
	    }
	    // between the obs and yz plane , back
	    len = std::abs(- bound - sphere_centers[i](0));
	    if (len <= sphere_radius[i]) continue;
	    dis = (len-sphere_radius[i])*0.5;
	    pos = sphere_centers[i];
	    pos(0) = -bound + dis;
	    if(!isPointinObs(pos))
	    {
	      pmids_vector.push_back(mid_info(pos,dis));
	    }
	    // between the obs and xz plane , left
	    len = std::abs(- bound - sphere_centers[i](1));
	    if (len <= sphere_radius[i]) continue;
	    dis = (len-sphere_radius[i])*0.5;
	    pos = sphere_centers[i];
	    pos(1) = -bound + dis;
	    if(!isPointinObs(pos))
	    {
	      pmids_vector.push_back(mid_info(pos,dis));
	    }
	    // between the obs and xz plane , right
	    len = std::abs(bound - sphere_centers[i](1));
	    if (len <= sphere_radius[i]) continue;
	    dis = (len-sphere_radius[i])*0.5;
	    pos = sphere_centers[i];
	    pos(1) = bound - dis;
	    if(!isPointinObs(pos))
	    {
	      pmids_vector.push_back(mid_info(pos,dis));
	    }

	    // between obs
	    
	    for(size_t j = i+1; j < obs_num; j++)
	    {
	      Eigen::Vector3d o2o = (sphere_centers[i] - sphere_centers[j]);
	      dis = o2o.norm() - sphere_radius[i] - sphere_radius[j];
	      pos = (dis*0.5+sphere_radius[j])*o2o.normalized()+sphere_centers[j];
	      if(!isPointinObs(pos))
		    {
		      pmids_vector.push_back(mid_info(pos,dis));
		    }

	    }
	 }
	  std::sort(pmids_vector.begin(), pmids_vector.end(),Mid_Greater);
	  for(size_t i = 0; i < pmids_vector.size(); i++)
	  {
		pmids_vector[i].index = i+1;
	  }
	  
	  	pmids_vector.insert(pmids_vector.begin(), mid_info(start,0,0));
		int goalind = pmids_vector.size();
		pmids_vector.insert(pmids_vector.end(), mid_info(goal,0,goalind));
		pmids_num = pmids_vector.size();
	  /*
	  for(auto i:pmids_vector)
	  {
	    std::cout << i.index<<" " << i.i.dis <<" " << i.pos(0) << " " << i.pos(1)<< " " << i.pos(2)<< std::endl; 
	  }
	  */
}


bool FMGPlanner::checkSegment(const Eigen::Vector3d& con, const Eigen::Vector3d& point)
{
  std::cout<<"&&&&&&&&&&&&&&&&&&& checking is free point:"<<point<<std::endl;

  double dis_cu2p = (point - con).norm();
  
  if(dis_cu2p < 0.002)
    return true;
  Eigen::Vector3d dir_cu2p = (point - con).normalized();
  //std::cout<<"&&&&&&&&&&&&&&&&&&& dir_cu2p:"<<dir_cu2p<<std::endl;

  for(size_t i =0; i < obs_num; i++)
  {
      //if((point - sphere_centers[i]).norm() < sphere_radius[i])
      //  return false;

      Eigen::Vector3d cur2obs = sphere_centers[i] - con;
      //std::cout<<"&&&&&&&&&&&&&&&&&&& cur2obs:"<<cur2obs<<std::endl;
      double proj = cur2obs.dot(dir_cu2p);
      //std::cout<<"&&&&&&&&&&&&&&&&&&& dis_cup:"<<dis_cu2p<<" proj: "<<proj<<std::endl;
      if(proj<0) continue;
      if(proj>dis_cu2p) continue;
      Eigen::Vector3d closestpoint = proj*dir_cu2p;
      std::cout << "closest point " << closestpoint << std::endl;
      //double closest_dis = cur2obs.dot(cur2obs) - proj*proj;
      double closest_dis = (sphere_centers[i] - closestpoint).norm();
      //std::cout<<"&&&&&&&&&&&&&&&&&&& closest_dis:"<<closest_dis<<std::endl;
      if(closest_dis<= sphere_radius[i]+to_obs)
        return false;
  }

  //std::cout << point(0)<<" " << point(1) << " " << point(2) << "is free!!";
  return true;

}

bool FMGPlanner::checkPoseCollision(const geometry_msgs::Pose & pose_msg, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs)
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
        	return true;
        }
    }
    else
    {
    	return false;
    }
      
}

bool FMGPlanner::checkCartPoseCollision(const Eigen::Vector3d &position, moveit_msgs::RobotState & init_state_msgs, moveit_msgs::RobotState &res_state_msgs)
{
	Eigen::Affine3d pose= Eigen::Affine3d::Identity();
  	pose.translation() = position;
  	geometry_msgs::Pose pose_msg = dr::toRosPose(dr::affineToIsometry(pose));
  	
  	pose_msg.orientation.x = fRand(-1,1);
   	pose_msg.orientation.y = fRand(-1,1);
   	pose_msg.orientation.z = fRand(-1,1);
   	pose_msg.orientation.w = fRand(-1,1);

   	return (checkPoseCollision(pose_msg,init_state_msgs,res_state_msgs))

}


//bool plan(int curr_ind,) //??????  start!!

bool FMGPlanner::plan(float checkgoal)
{
	std::mt19937_64 rng;
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
    rng.seed(ss);
    // initialize a uniform distribution between 0 and 1
    std::uniform_real_distribution<double> unif(0, 1);
	
	GenerateGaps();
	int curr_ind = -1;
	double randomnum = unif(rng);
	randomnum = 0;
	double cur2goal_dis = (currentpos - goal).norm();
	while(currentpos != goal)
	{
		if(randomnum<checkgoal)
		{
			//check goal
			if(checksegment(currentpos, goal))
			{
				std::cout << "goal is reaachable!!!" << std::endl;
      			//AddMidPoint((currentpos+goal)*0.5);
      			AddMidPoint(mid_info(goal));
      			return true;
			}
		}

		bool foundonemid = false;
		for(size_t i = 0; i < pmids_num; i++)
		{
			if(checksegment(currentpos, pmids_vector[i].pos))
			{
				// near to goal
				double p2goal_dis = (pmids_vector[i].pos - goal).norm()
				if(cur2goal_dis > p2goal_dis)
				{
					foundonemid = true;
					AddMidPoint(pmids_vector[i]);
					currentpos = pmids_vector[i].pos;
					curr_ind = i;
					cur2goal_dis = p2goal_dis;
					break;
				}
			}
		}
		if(!foundonemid)
		{
			std::cout<<"No Mid Point Found! "<<std::endl;
			return false;
		}
	}
	return true;
}

bool FMGPlanner::setconnet()
{
	size_t mid_num = pmids_vector.size(); // the number of mid points including start and goal
	if(mid_num<2)
	{
		ROS_ERROR("mid_num < 2. In the function setconnect ");
		return false;
	}
	//initialize to zero
	std::vector<bool> onerow;
	onerow.resize(mid_num,0);
	connected.resize(mid_num,onerow);
	std::vector<int> onenode{0};
	searchnode.resize(mid_num,onenode);
	searchnode_pointer_cur.resize(mid_num,0);

	for(size_t i = 0; i < mid_num; i++)
	{
		//connected[i][i] = 0;
		for(size_t j = i+1; j< mid_num; j++)
		{
			if(checkSegment(pmids_vector[i].pos, pmids_vector[j].pos))
			{
				connected[i][j] = 1;
				connected[j][i] = 1;
				searchnode[i].push_back(j);
				searchnode[j].push_back(i);
				//draw_line(pmids_vector[i].pos,pmids_vector[j].pos);
			}		

		}
	}
	return true;
}
bool FMGPlanner::plantraj(int nodenum, std::vector<int> traj, std::vector<int> searchnode_rec)
{
	//std::cout<<" plantraj now nodenum: "<< nodenum<<std::endl;
	if(nodenum < 0) return false;
	if(traj.size() < nodenum)
	{
		std::cout <<traj.size()<<" "<< nodenum<< " traj num problem!!! traj, nodenum " <<std::endl;
		return false;
	}
	if(traj[nodenum] == pmids_vector.size()-1)
	{
		std::cout << "reach goal!" <<std::endl;

		traj_mid_index = traj;
		searchnode_pointer_cur = searchnode_rec;
		return true;
	}
	else
	{
		if(searchnode.size()+1 < traj[nodenum])
		{
			std::cout<<"fail!! searchnode number" << searchnode_rec.size()<<" " <<traj[nodenum] <<std::endl;
			return false;
		}
		const std::vector<int> &cur_node = searchnode[traj[nodenum]];
		int node_len = cur_node.size();

		if(node_len<2 || searchnode_rec[traj[nodenum]] >= node_len-1)
		{
			// this node fails
			std::cout << "back!!!!" << endl;
			traj.pop_back();
			searchnode_rec[traj[nodenum]] = 0;
			return plantraj_v2(nodenum-1,traj,searchnode_rec);
		}
		else
		{
			if(nodenum==0)
			{ // start point

				for(size_t i = searchnode_rec[0]+1; i< node_len; i++)
				{
					searchnode_rec[0] = i;
					cout <<" search in node "<<traj[nodenum] << std::endl;
					
						if(cur_node[i] != pmids_vector[cur_node[i]].index)
						{
							std::cout <<"problem!!! 654 line "<<std::endl;
							return false;
						}
						Eigen::Vector3d pos = pmids_vector[cur_node[i]].pos;
						//printvpoint(start);
						//printvpoint(pos);

						traj.push_back(cur_node[i]);
						for(int k =0; k < traj.size(); k++)
						{
							std::cout << traj[k] << " ";
						}
						std::cout << std::endl;
						return plantraj(nodenum+1,traj,searchnode_rec);
							
					
				}
			}
			else 
			{
				Eigen::Vector3d curpos = pmids_vector[traj[nodenum]].pos;
				bool foundone = false;
				for(size_t i = searchnode_rec[traj[nodenum]]+1; i< node_len; i++)
				{
					searchnode_rec[traj[nodenum]] = i;

					cout <<" search in node " << traj[nodenum] <<" "<< i<< std::endl;
					Eigen::Vector3d pos = pmids_vector[cur_node[i]].pos;
					if((pos-goal).norm() < (curpos - goal).norm())
					{
						//printvpoint(curpos);
						//printvpoint(pos);
						//std::cout <<"compare distantce"<< norm(pos-goal) << " "<< norm(curpos-goal) << std::endl;
						if(cur_node[i] != pmids_vector[cur_node[i]].index)
						{
							std::cout <<"problem!!! 654 line "<<std::endl;
							return false;
						}
						if(improve)
						{
							while(nodenum>1)
							{
								cout << nodenum<<endl;
								Eigen::Vector3d backpos = pmids_vector[traj[nodenum-1]].pos;
								if(connected[traj[nodenum-1]][cur_node[i]])
								{
									cout << "improved!!!! nodenm: "<< nodenum << endl;
									nodenum = nodenum-1;
									traj.pop_back();

								}
								else
								{
									break;
								}
							}
								

						}
					
						traj.push_back(cur_node[i]);
						for(int k =0; k < traj.size(); k++)
						{
							std::cout << traj[k] << " ";
						}
						std::cout << std::endl;
						foundone = true;
						return plantraj(nodenum+1,traj,searchnode_rec);
							
					}
				}
				if(!foundone)
				{
					std::cout<<"no found ! back!"<<std::endl;
					traj.pop_back();
					searchnode_rec[traj[nodenum]] = 0;
					return plantraj_v2(nodenum-1,traj,searchnode_rec);
				}
			}
			
		}

	}
	std::cout <<"plantraj end???" <<std::endl;
	return true;
}

bool FMGPlanner::replan_v2(int replan_startpoint, std::vector<int> oldtraj,std::vector<int> searchnode_rec)
{
	//int nodeind = oldtraj[replan_startpoint+1];
	for(size_t i = replan_startpoint+1; i < oldtraj.size(); i++)
	{
		searchnode_rec[oldtraj[i]] = 0;
	}
	oldtraj.erase(oldtraj.begin()+replan_startpoint+1, oldtraj.end());
	bool replan_suc = plantraj_v2(replan_startpoint,oldtraj,searchnode_rec);
	if(!replan_suc) 
	{
		ROS_ERROR("Replan failed!!!");
	}
	else
		return replan_suc;

}

void FMGPlanner::indextoTraj()
{
	//traj_mid_index
	if(traj_mid_index.size()<2)
	{
		return;
	}
	else
	{
		if(traj_mid_index[0] != 0)
		{
			ROS_ERROR("The Generated index set is not valid !");
			return ;
		}
		for(size_t i = 1; i < traj_mid_index.size(); i++)
		{
			Traj_mid_pos.push_back(pmids_vector[traj_mid_index[i]]);
		}
	}
}

bool FMGPlanner::Traj_interp_tomsgs()
{
	bool interp_suc = fmgplanner::cubic_interp(interpTraj, initpTraj);
	if(!interp_suc)
	{
		ROS_ERROR_STREAM("Interpolation failed!!!");
		return false;
	}
	moveit_msgs::RobotTrajectory joint_solution = fmgplanner::toROSJointTrajectory(interpTraj, joint_names, 1.0);
  	ROS_INFO("Visualizing the trajectory");
  	display_trajectory.trajectory.push_back(joint_solution);
  	display_publisher.publish(display_trajectory);
  	return true;
  
}

bool FMGPlanner::GetMidIKSolution()
{
	size_t traj_mid_size = Traj_mid_pos.size();
	if(traj_mid_size <2)
	{
		ROS_ERROR_STREAM("Mid Traj Size < 2 !!");
		return false;
	}
	std::vector<double> joint_values;
	moveit_msgs::RobotState moveit_init;
	moveit_msgs::RobotState moveit_res;
	// copy the start state 
  	moveit::core::robotStateToRobotStateMsg(robot_state_,moveit_init);
	for(size_t i = 0; i < traj_mid_size; i++)
	{
		const mid_info onemid& = Traj_mid_pos[i];
		if(checkCartPoseCollision(onemid.pos, moveit_init,moveit_res))
		{
			moveit::core::robotStateMsgToRobotState(moveit_res, robot_state_);
           	robot_state_.copyJointGroupPositions(joint_model_group, joint_values);
			visual_tools_.publishRobotState(robot_state_, rviz_visual_tools::GREEN);
			moveit_rs = moveit_res;
			initpTraj.push_back(joint_values);
			
		}
		else
		{
			ROS_ERROR_STREAM("Mid Traj Size < 2 !!");
			std::cout << "the "<< i <<" th mid point IK failed!!"<<std::endl;
			std::cout << onemid.pos<<endl;
		}

	}


}

void FMGPlanner::test()
{
	init();
	loadObs("obs.scene",true);
	GenerateGaps();

	if(plan(1)) //get Traj_mid_pos 
	{
		if(GetMidIKSolution()) // get interpTraj
		{
			Traj_interp_tomsgs();
		}
	}

}

void FMGPlanner::test_v2()
{
	init();
	loadObs("obs.scene",true);
	GenerateGaps();
	setconnet();

	std::vector<int> traj_test;
	// add the start point
	traj_test.push_back(0);
	std::vector<int> searchnode_rec;
	searchnode_rec = env.searchnode_pointer_cur;
	if(plantraj(0,traj_test,searchnode_rec))
	{
		indextoTraj();
		if(GetMidIKSolution()) // get interpTraj
		{
			Traj_interp_tomsgs();
		}

	}



}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fmgplanner init");
	FMGPlanner planner;
	planner.test();

	ros::shutdown();

  	return 0;
}

//void FMGPlanner::DisplayRobotState()
//{

	//visual_tools_.publishRobotState(robot_state, rviz_visual_tools::GREEN);
//}

