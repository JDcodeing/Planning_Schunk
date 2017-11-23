#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <functional>
#include <random>
#include <queue>
#include <vector>
#include "spline.h"

//g++ Env2d.cpp $(pkg-config --libs --cflags opencv) -o Env2d
using namespace std ;

using namespace cv;
Mat img ;
const double eps = 1e-5;

class mid_info
{
	
public:
	Point2d pos;
	double dis;

	mid_info(Point2d p, double d):pos(p), dis(d){};
	
};


bool disgreater( const mid_info& a, const mid_info& b)  { return a.dis > b.dis; }


class Env2d
{
public:
	Env2d(int wid, int height, double start1, double start2, double goal1, double goal2)
							:env_wid(wid),
							env_height(height),
							obs_num(5),
							currentpos(Point2d(start1,start2)),
							goal(Point2d(goal1,goal2)),
							start(currentpos)
	{
		centers.clear();
		radius.clear();
		initTraj.clear();
		interpTraj.clear();
		
		cur2goal_dis = norm(currentpos - goal);

		img = cv::Mat::zeros(wid,height, CV_64FC1);
		img.setTo(cv::Scalar(255,255,255));
		
		centers.push_back(Point2d(50.5,300-50.5));
		radius.push_back(15);
		centers.push_back(Point2d(150.0,300-80.0));
		radius.push_back(25);
		centers.push_back(Point2d(200.0,300-150.0));
		radius.push_back(30);
		centers.push_back(Point2d(93.0,300-209.0));
		radius.push_back(27);
		centers.push_back(Point2d(95.0,196.0));
		radius.push_back(20);

		for(size_t i = 0; i < centers.size(); i++)
		{
			draw_circle(centers[i], radius[i]-5,Scalar(0,0,0));
			//Point2d pos(centers[i].x, height-centers[i].y);
			//circle(img, pos, radius[i], Scalar(0,0,0), CV_FILLED, 8);	
		}
		
		cv::imwrite("./result/Env2.png", img);

	};
	~Env2d(){};

	void GenerateGaps();
	void draw_midpoints();
	void draw_traj();
	void draw_interptraj();
	void draw_circle(const Point2d& pos, int rad, Scalar color)
	{
		Point2d draw_pos(pos.x, env_height-pos.y);
		circle(img, draw_pos, rad, color, CV_FILLED, 8);
	}
	void draw_line(const Point2d& p1, const Point2d& p2)
	{
		Point2d draw_pos1(p1.x, env_height-p1.y);
		Point2d draw_pos2(p2.x, env_height-p2.y);
		line(img,draw_pos1, draw_pos2, Scalar(0,0,255), 1, 8);

	};
	bool isPosFree(const Point2d&);
	bool plan();
	bool forwardexist(const Point2d&);
	bool isRobotFree(const Point2d&);
	bool isReachable(const Point2d&);
	void AddMidPoit(const Point2d&);
	bool TryGoal();
	bool cubic_interp();

private:
	Mat img;
	int env_wid;
	int env_height;
	int obs_num;
	vector<Point2d> centers;
	vector<Point2d> draw_centers;

	vector<int> radius;
	//priority_queue<mid_info, vector<mid_info>, Compare> pids_que;
	vector<mid_info> pmids_vector;
	Point2d currentpos;
	const Point2d goal;
	const Point2d start;
	vector<Point2d> initTraj;
	vector<Point2d> interpTraj;
	double cur2goal_dis;

	/* data */
};

void Env2d::GenerateGaps()
{
	for(size_t i =0; i < obs_num; i++)
	{
		// dis to the boundary
		//left
		double dis(centers[i].x - radius[i]);
		Point2d pid(dis*0.5,centers[i].y);
		pmids_vector.push_back(mid_info(pid,dis));
		//right
		dis = env_wid - centers[i].x - radius[i];
		pid.x = env_wid - dis*0.5;
		pmids_vector.push_back(mid_info(pid,dis));
		//down
		dis = centers[i].y - radius[i];
		pid.x = centers[i].x;
		pid.y = dis*0.5;
		pmids_vector.push_back(mid_info(pid,dis));
		//up
		dis = env_height - centers[i].y - radius[i];
		pid.y = env_height - dis*0.5;
		pmids_vector.push_back(mid_info(pid,dis));

		//dis to other obstacle
		if(i == obs_num-1) 
			break;
		for(size_t j = i+1; j < obs_num; j++)
		{
			dis = norm(centers[i] - centers[j]);
			pid = (centers[i] + centers[j])*0.5;
			pmids_vector.push_back(mid_info(pid,dis));
			//circle(img, pid, 2, Scalar(0,0,255), CV_FILLED, 8);	
		}
		


	}

	std::sort(pmids_vector.begin(), pmids_vector.end(),disgreater);
	for(auto i:pmids_vector)
	{
		cout << i.dis <<" " << i.pos.x << " " << i.pos.y<< endl; 
	}

}
void Env2d::draw_midpoints()
{
	for(auto ele : pmids_vector)
	{
		//const mid_info& mid = pids_que.top(); 
		Point2d& ele_pos = ele.pos;
		draw_circle(ele_pos,2,Scalar(0,0,0));
		//cout << ele.dis << endl;
		//pids_que.pop();
		
	}
	cv::imwrite("./result/Env_with_mids2.png", img);
	
}
void Env2d::draw_traj()
{
	for(auto it=initTraj.begin(); it!=initTraj.end()-1; ++it)
	{
		//const mid_info& mid = pids_que.top(); 
		Point2d& ele_pos = *it;
		draw_circle(ele_pos,2,Scalar(255,0,0));
		draw_line(*it, *(it+1));
		cout << it->x << " " << it->y << endl;
		//pids_que.pop();
		
	}
	Point2d& ele_pos = initTraj.back();
	draw_circle(ele_pos,2,Scalar(255,0,0));
	cv::imwrite("./result/Env_with_initTraj2.png", img);
	
}
void Env2d::draw_interptraj()
{
	for(auto it=interpTraj.begin(); it!=interpTraj.end()-1; ++it)
	{
		//const mid_info& mid = pids_que.top(); 
		Point2d& ele_pos = *it;
		draw_circle(ele_pos,2,Scalar(255,0,0));
		draw_line(*it, *(it+1));
		cout << it->x << " " << it->y << endl;
		//pids_que.pop();
		
	}
	Point2d& ele_pos = interpTraj.back();
	draw_circle(ele_pos,2,Scalar(255,0,0));
	cv::imwrite("./result/Env_with_interpTraj2.png", img);
	
}
bool Env2d::isPosFree(const Point2d& point)
{
	cout << "*****is pos free : "<< point.x << " " << point.y << endl;

	double dis_cu2p(norm(point - currentpos));
	Point2d dir = (point - currentpos) * (1/dis_cu2p); 
	for(size_t i = 0; i < obs_num; i++)
	{

		// point is in the circle
		if(norm(point-centers[i]) < radius[i])
			return false;

		// compute the euclidean distance between A and B
		
		cout <<"***********"<<i << "th obs: " <<centers[i].x <<" " << centers[i].y<<endl;
		if (dis_cu2p<eps)
			continue;
		// compute the direction vector D from A to B
		//???
		//cout << dir.x <<" "<< dir.y << endl;
		Point2d cur2obs = centers[i] - currentpos ;
		// the projection 
		double t = cur2obs.dot(dir);
		double tmp = cur2obs.dot(point - currentpos)/dis_cu2p;
		cout << "cur2obs"<< cur2obs.x << " " << cur2obs.y<< endl;
		cout << "dir"<< dir.x << " " << dir.y<< endl;
		cout << "t: " << t <<" " << tmp<< endl;
		cout << "dis_cu2p: " << dis_cu2p << endl;
		
		if(t<0) continue;
		if(t>dis_cu2p) continue;
		// the distance between obs and line
		Point2d closest_point(t*dir);
		cout <<"the clos point is : "<< closest_point.x <<" "<< closest_point.y << endl;
		if(norm(closest_point-centers[i]) <= radius[i]) 
			return false;

	}
	cout << "is free!!" << endl;
	return true;
}
bool Env2d::forwardexist(const Point2d& point)
{
	//todo
	return true;
}
bool Env2d::isRobotFree(const Point2d& point)
{
	//todo
	return true;
}

bool Env2d::isReachable(const Point2d& point)
{
	if(!isPosFree(point))
		return false;
	if(cur2goal_dis < norm(point - goal))
		return false;
	if(!forwardexist(point))
		return false;
	if(!isRobotFree(point))
		return false;

	return true;
}
bool Env2d::TryGoal()
{
	if (isReachable(goal))
		return true;
	return false;
}
void Env2d::AddMidPoit(const Point2d& point)
{
	cout << " *********Add a mid point *****" << endl;
	cout << point.x << " " << point.y << endl;
	initTraj.push_back(point);	
	currentpos = point;
	cur2goal_dis = norm(currentpos - goal);

}
bool Env2d::plan()
{
	initTraj.push_back(start);
	while(currentpos != goal)
	{
		if(TryGoal())
		{
			cout << "goal is reaachable!!!" << endl;
			AddMidPoit(goal);
			return true;
		}
		bool foundonemid = false;
		
		for(auto it = pmids_vector.begin(); it != pmids_vector.end(); ++it)
		{
			const Point2d p_mid= it->pos;
			if(isReachable(p_mid))
			{
				foundonemid = true;
				AddMidPoit(p_mid);
				break;
			}

		}
		if(!foundonemid)
		{
			cout << "Mid Point Not Found " << endl;
			return false;
		}
	}
	return true;
}

bool Env2d::cubic_interp()
{
	//size_t pos_len = 0;
	if(!(initTraj.size()>1))
		return false;
	
	
	interpTraj.clear();
	
	std::vector<double> X,Y;
	for(auto it = initTraj.begin(); it!=initTraj.end(); it++)
	{
		X.push_back(it->x);
		Y.push_back(it->y);
	}
	tk::spline s;
	s.set_points(X,Y);

	int interpnum = norm(start-goal)/3;
	double stepsize = abs(goal.x - start.x)/interpnum;
	for(int k =0; k<interpnum; k++)
	{
			
			double val  = stepsize*k;
			interpTraj.push_back(Point2d(val, s(val)));
	}
	interpTraj.push_back(goal);
	
	return true;
}

int main()
{
	Env2d env(300,300,0,0,300,300);
	env.GenerateGaps();
	//env.draw_midpoints();
	//env.plan();
	//env.cubic_interp();
	//env.draw_traj();
	//env.draw_interptraj();
	
	
	//env.draw_midpoints();
	return 0;

}