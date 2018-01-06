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

#include "eigen_ros.hpp"

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

Eigen::Vector3d Vector3dRand(const Eigen::Vector3d &vec, double min, double max)
{
  Eigen::Vector3d res;
  for(int i = 0; i < 3; i++)
  {
    res[i] = vec[i] + fRand(min,max);
  }
  return res;
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


  bool cubic_interp_old(std::vector<std::vector<double> >& result, 
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
  double max_diff(const std::vector<double> &next_vec, const std::vector<double> &vec)
  {
    size_t len = vec.size();
    double maxdiff = 0, tmp;
    for(unsigned i = 0; i < len; i++)
    {
      tmp = std::abs(next_vec[i] - vec[i]);
      if(tmp > maxdiff) maxdiff = tmp;
    }
    return maxdiff;
  }
  bool cubic_interp(std::vector<std::vector<double> >& result, 
            const std::vector<std::vector<double> >& pidpoints, const double max_diff_step)
  {
    size_t pos_len = 0;
    size_t pointsize = pidpoints.size();
    if(pidpoints.size()>1)
      pos_len = pidpoints[0].size();
    else
      return false;
    //if(pos_len != 6) return false;
    if(!result.empty())
      result.clear();

    // get the max diff 
    double maxdiff;
    int step = 0;
    std::vector<double> index;
    index.reserve(pointsize);
    index.push_back(0);
    for(size_t i = 1; i < pointsize; i++)
    {
      maxdiff = max_diff(pidpoints[i],pidpoints[i-1]);
      step += (int)floor(maxdiff/max_diff_step+0.5);
      index.push_back(step);
    }

    result.resize(index.back()+1,vector<double>(6,0));
    for(size_t i = 0; i < pos_len; i++)
    {
      std::vector<double> Y;
      
      for( size_t j = 0; j < pointsize; j++)
      {
        Y.push_back(pidpoints[j][i]) ;
      }
      tk::spline s;
      size_t k;
      if(pointsize<=2)
        s.set_points(index,Y,false); // linear interpolation
      else
        s.set_points(index,Y,true); // cubic interpolation

      for(k =0; k<index.back()+1; k++)
      {
        result[k][i] = s(k);
      }
     
    }
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

  void interpolate(const Eigen::Vector3d &from, const Eigen::Vector3d &to, const double t, Eigen::Vector3d &state) 
{
    state = from + (to-from)*t;
}

void interpolateCartesianPath(std::vector<Eigen::Vector3d> &CartesianPath, const double stepsize_cart)
{
    if(CartesianPath.size() <2) 
    return;
    std::vector<Eigen::Vector3d> newPath(1, CartesianPath[0]);

    for(unsigned int i = 1; i < CartesianPath.size(); ++i)
    {
        double distance  = (CartesianPath[i] - CartesianPath[i-1]).norm();
        if(distance < std::numeric_limits<double>::epsilon()) continue;
        int step = distance/stepsize_cart; // floor (+0.5)
        double fic = 1.0/step;

        Eigen::Vector3d temp = Eigen::Vector3d(0,0,0);
        for(unsigned int j = 0; j < step; ++j)
        {
            interpolate(CartesianPath[i-1], CartesianPath[i], fic*(1+j), temp);
            newPath.push_back(temp);
        }
    }

    CartesianPath.assign(newPath.begin(), newPath.end());

}



void toROSPoseVec(const std::vector<Eigen::Vector3d> posvec, std::vector<geometry_msgs::Pose> PoseMsgs)
{
  Eigen::Isometry3d eigenpose= Eigen::Isometry3d::Identity();
  geometry_msgs::Pose temp;
  for(unsigned int i = 0; i < posvec.size(); i++)
  {
    eigenpose.translation() = posvec[i];
    temp = fmg::toRosPose(eigenpose);
    PoseMsgs.push_back(temp);
  }

}
 /* void printJointState(const robot_state::RobotState &rs)
  {
    std::vector<double> jointvalues;
    rs.getStateValues(jointvalues);
    ROS_INFO("Q1: %f, Q2: %f, Q3: %f, Q4: %f, Q5: %f, Q6: %f",
    jointvalues[0],jointvalues[1],jointvalues[2],jointvalues[3],jointvalues[4],jointvalues[5]);
  }*/

} // namespace