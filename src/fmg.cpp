#include <vector>
#include "FMGPlanner.h"
#include <iostream>
#include <ros/ros.h>

int main()
{
	ros::init(argc, argv, "fmgplanner init");
	FMGPlanner planner;
	planner.init();
	planner.loadObs("obs.scene",true);
	planner.GenerateGaps();

	if(planner.plan(1)) //get Traj_mid_pos 
	{
		if(planner.GetMidIKSolution()) // get interpTraj
		{
			planner.Traj_interp_tomsgs();
		}
	}


	ros::shutdown();

  	return 0;


}