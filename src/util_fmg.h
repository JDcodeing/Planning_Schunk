#pragma once

#include <vector>
#include <map>
#include "spline.h"
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}








namespace fmgplanner
{
  void printtraj(const std::vector<std::vector<double> >& js)
  {
    std::cout <<"traj!!!!!!!!!---------------- "<<std::endl;
    for(int i =0; i < js.size(); i++)
    {

       for(auto j: js[i])
      {
        std::cout << j<<" ";
      }
      std::cout << std::endl;
    }
  }

  template<class T>
  void printvec1d(const std::vector<T>& vec)
  {
    for(T j: vec)
    {
      std::cout <<j<<" ";
    }
    std::cout << std::endl;
  }


  bool cubic_interp(std::vector<std::vector<double> >& result, 
            const std::vector<std::vector<double> >& pidpoints)
  {
    size_t pos_len = 0;
    if(pidpoints.size()>1)
      pos_len = pidpoints[0].size();
    else
      return false;
    //if(pos_len != 6) return false;
    
    result.clear();
    for(size_t k = 0; k < (pidpoints.size()-1)*100+1; k++)
    {
      std::vector<double> pos(6,0.0);
      result.push_back(pos);
    }
    std::cout <<"result size : "<< result.size()<<std::endl;

    for(size_t i = 0; i < pos_len; i++)
    {
      std::vector<double> X,Y;
      std::map<double, double> points;


      for( size_t j = 0; j < pidpoints.size(); j++)
      {
        points[j] = pidpoints[j][i];
      }

     
      for(std::map<double,double>::iterator it = points.begin(); it != points.end(); ++it) 
      {
          X.push_back(it->first);
          //cout << it->first << endl;
          Y.push_back(it->second);
      }

      tk::spline s;
      s.set_points(X,Y);
      size_t k;

      for(k =0; k<(pidpoints.size()-1)*5+1; k++)
      {

        double val  = 0.2*k;
        //std::cout << k << " " ;
        result[k][i] = s(val);
      }
     // std::cout << std::endl;
    }
    //printtraj(pidpoints);
    //printtraj(result);
    return true;

  }

  moveit_msgs::RobotTrajectory toROSJointTrajectory(const std::vector<std::vector<double> >& points,
                      const std::vector<std::string>& joint_names,
                      double time_delay)
  {
    moveit_msgs::RobotTrajectory result;
    result.joint_trajectory.header.stamp = ros::Time::now();
    result.joint_trajectory.header.frame_id = "/world";
    result.joint_trajectory.joint_names = joint_names;

    double time_offset = 0.0;
    for(auto it = points.begin(); it!= points.end(); ++it)
    {
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.positions = *it;
      pt.velocities.resize((*it).size(), 0.0);
      pt.accelerations.resize((*it).size(), 0.0);
      pt.effort.resize((*it).size(), 0.0);
      // set the time into the trajectory
      pt.time_from_start = ros::Duration(time_offset);
      // increment time
      time_offset += time_delay;

      result.joint_trajectory.points.push_back(pt);
      
    }
    return result;

  }

  void printJointState(const sensor_msgs::JointState &js)
  {
    ROS_INFO("Q1: %f, Q2: %f, Q3: %f, Q4: %f, Q5: %f, Q6: %f",
    js.position.at(0), js.position.at(1),js.position.at(2),js.position.at(3),js.position.at(4),js.position.at(5));

  }

 /* void printJointState(const robot_state::RobotState &rs)
  {
    std::vector<double> jointvalues;
    rs.getStateValues(jointvalues);
    ROS_INFO("Q1: %f, Q2: %f, Q3: %f, Q4: %f, Q5: %f, Q6: %f",
    jointvalues[0],jointvalues[1],jointvalues[2],jointvalues[3],jointvalues[4],jointvalues[5]);
  }*/

} // namespace