bool checkStateCollision(std::vector<double> &values){
	bool res = false;
		
		//moveit_msgs::PlanningScene ps_msg; 
		//ps_msg.robot_state.is_diff = true;
  	//planning_scene->getPlanningSceneMsg(ps_msg);
  	//planning_scene->setPlanningSceneMsg(ps_msg);
  	//planning_scene_diff_publisher.publish(ps_msg);
  	
  		collision_detection::CollisionRequest collision_request;
 		collision_detection::CollisionResult collision_result;
 		
 		robot_state::RobotState& current_state_r = planning_scene_->getCurrentStateNonConst();
 		collision_request.group_name = "manipulator";
 		//collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
 		
 		collision_request.contacts = true;
		collision_request.max_contacts = 100;
		
 		std::vector<double> joint_values;
 		
 		current_state_r.copyJointGroupPositions(joint_model_group_, joint_values);
		joint_values[0] = values[0];
		joint_values[1] = values[1];
		joint_values[2] = values[2];
		joint_values[3] = values[3];
		joint_values[4] = values[4];
		joint_values[5] = values[5];
		current_state_r.setJointGroupPositions(joint_model_group_, joint_values);
		planning_scene_->setCurrentState(current_state_r); 
		
		collision_result.clear();
		
		planning_scene->checkCollision(collision_request, collision_result, current_state_r);
		/*ROS_INFO_STREAM("Test 4: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " collision");
		collision_detection::CollisionResult::ContactMap::const_iterator it;
		for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it){
  		ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
		}*/
		
    if(collision_result.collision){
    	res = true;
    }
		return res;
	}
};