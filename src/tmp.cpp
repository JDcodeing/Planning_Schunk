bool isStateValid(const planning_scene::PlanningScene *planning_scene,robot_state::RobotState *state,
					const robot_state::JointModelGroup *group, const std::vector<double> joint_gourp_variable_values)
{
	state->setJointGroupPositions(group, joint_gourp_variable_values);
    //std::vector<double> joint_values;
    //state->copyJointGroupPositions(group, joint_values);
    collision_detection::CollisionRequest request;
    request.verbose = false;
    request.group_name = group->getName();
    collision_detection::CollisionResult result;
    planning_scene->checkCollision(request, result, *state);

    if (result.collision)
        return false;
   
    return planning_scene->isStateFeasible(*state);
}
