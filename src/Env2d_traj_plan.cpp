#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <functional>
#include <random>
#include <queue>
#include <vector>
#include <map>
#include <cmath>
#include "spline.h"

//g++ -std=c++11 Env2d_traj_plan.cpp $(pkg-config --libs --cflags opencv) -o Env2d_traj_plan
using namespace std ;

using namespace cv;
Mat img ;
const double eps = 1e-5;

class mid_info
{
	
public:
	Point2d pos;
	double dis;
	int index;

	mid_info(Point2d p, double d):pos(p), dis(d), index(-1){};
	mid_info(Point2d p, double d, int ind):pos(p), dis(d), index(ind){};
	
};


bool disgreater( const mid_info& a, const mid_info& b)  { return a.dis > b.dis; }


class Env2d
{
public:
	Env2d(int wid, int height, double start1, double start2, double goal1, double goal2, bool improved_val)
							:env_wid(wid),
							env_height(height),
							//obs_num(10),
							currentpos(Point2d(start1,start2)),
							goal(Point2d(goal1,goal2)),
							start(currentpos),
							improved(improved_val)
	{
		centers.clear();
		radius.clear();
		initTraj.clear();
		interpTraj.clear();
		pmids_vector.clear();
		interpTrajlen_ang.clear();
		initlen_ang.clear();
		searchnode.clear();

		
		
		cur2goal_dis = norm(currentpos - goal);

		img = cv::Mat::zeros(wid,height, CV_64FC1);
		img.setTo(cv::Scalar(255,255,255));
		
		centers.push_back(Point2d(50.5,300-50.5));
		radius.push_back(15);
		centers.push_back(Point2d(95.0,196.0));
		radius.push_back(20);
		centers.push_back(Point2d(170.0,300-80.0));
		radius.push_back(25);
		centers.push_back(Point2d(130.0,300-144.0));
		radius.push_back(25); 
		centers.push_back(Point2d(90.0,300-209.0));
		radius.push_back(27);
		centers.push_back(Point2d(200.0,300-150.0));
		radius.push_back(30);
		centers.push_back(Point2d(170.0,300-200.0));
		radius.push_back(30); 
		
		centers.push_back(Point2d(230.0,300-100.0));
		radius.push_back(16);

		centers.push_back(Point2d(230.0,300-200.0));
		radius.push_back(20);

		centers.push_back(Point2d(60.0,300-150.0));
		radius.push_back(15);


		obs_num = centers.size();
		
		

		for(size_t i = 0; i < centers.size(); i++)
		{
			draw_circle(centers[i], radius[i]-5,Scalar(0,0,0));
			//Point2d pos(centers[i].x, height-centers[i].y);
			//circle(img, pos, radius[i], Scalar(0,0,0), CV_FILLED, 8);	
		}
		
		cv::imwrite("../result/2d/Envlen_angle.png", img);

	};
	~Env2d(){};

	bool GenerateGaps();
	bool GenerateGaps_dynamic(Point2d current);
	void draw_midpoints();
	void draw_traj();
	void draw_interptraj();
	void draw_circle(const Point2d& pos, int rad, Scalar color)
	{
		Point2d draw_pos(pos.x, env_height-pos.y);
		//cout << "draw : " << draw_pos <<endl;
		circle(img, draw_pos, rad, color, CV_FILLED, 8);
	}
	void draw_line(const Point2d& p1, const Point2d& p2)
	{
		Point2d draw_pos1(p1.x, env_height-p1.y);
		Point2d draw_pos2(p2.x, env_height-p2.y);
		line(img,draw_pos1, draw_pos2, Scalar(0,0,255), 1, 8);

	};
	bool draw_mid_traj(const std::vector<int> &traj);
	bool isPosFree(const Point2d&);
	bool isPosinObs(const Point2d& );
	bool plan();
	bool plantraj(int nodenum, std::vector<int> traj, vector<vector<int> > searchnode);

	bool plantraj_v2(int nodenum, std::vector<int> traj, std::vector<int> searchnode_rec);
	void replan(int replan_startpoint, std::vector<int> oldtraj,std::vector<std::vector<int> > searchnode_rec);
	void replan_v2(int replan_startpoint, std::vector<int> oldtraj,std::vector<int> searchnode_rec);
	bool forwardexist(const Point2d&);
	bool isRobotFree(const Point2d&);
	bool isReachable(const Point2d&);
	void AddMidPoit(const Point2d&);
	bool TryGoal();
	bool cubic_interp(int);
	bool cubic_interp_seveal(vector<Point2d>&);
	void changecoord(vector<Point2d>&, vector<Point2d>&);
	void changeback(vector<Point2d>&, vector<Point2d>&);
	bool setconnet();
	bool checkSegment(const Point2d& start, const Point2d& point);

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
	vector<int> traj_mid_index;
	vector<Point2d> initTraj;
	vector<Point2d> initlen_ang;
	vector<Point2d> interpTrajlen_ang;
	vector<Point2d> interpTraj;
	vector<vector<bool> > connected;
	vector<vector<int> > searchnode;
	vector<int> searchnode_pointer_cur;
	double cur2goal_dis;
	bool improved;

	/* data */
};
bool Env2d::setconnet()
{
	size_t mid_num = pmids_vector.size();
	if(mid_num<2)
	{
		std::cout<<"mid_num < 2"<<std::endl;
		return false;
	}
	
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


	/*for(int i = 0; i < searchnode.size();i++)
	{
		for(int j = 0; j < searchnode[i].size(); j++)
			std::cout << searchnode[i][j] << " ";
		std::cout << std::endl;

	}
	*/

    return true;
}
bool Env2d::GenerateGaps()
{

	for(size_t i =0; i < obs_num; i++)
	{
		// dis to the boundary
		//left
		double dis(centers[i].x - radius[i]);
		Point2d pid(dis*0.5,centers[i].y);
		if(!isPosinObs(pid))
			pmids_vector.push_back(mid_info(pid,dis));
		//right
		dis = env_wid - centers[i].x - radius[i];
		pid.x = env_wid - dis*0.5;
		if(!isPosinObs(pid))
			pmids_vector.push_back(mid_info(pid,dis));
		//down
		dis = centers[i].y - radius[i];
		pid.x = centers[i].x;
		pid.y = dis*0.5;
		if(!isPosinObs(pid))
			pmids_vector.push_back(mid_info(pid,dis));
		//up
		dis = env_height - centers[i].y - radius[i];
		pid.y = env_height - dis*0.5;
		if(!isPosinObs(pid))
			pmids_vector.push_back(mid_info(pid,dis));

		//dis to other obstacle
		
		for(size_t j = i+1; j < obs_num; j++)
		{
			//cout << "between "<<i<<" and "<<j<<endl;
			dis = norm(centers[i] - centers[j]);

			Point2d dir = (centers[i] - centers[j])*(1.0/dis);
			dis = dis - radius[i]-radius[j];
			//cout << "dir: "<< dir<<endl;
			pid = centers[j] + (dis*0.5+radius[j])*dir;
			//pid = (centers[i] + centers[j])*0.5;
			//cout <<"pid: " << pid << endl;
			if(!isPosinObs(pid))
				pmids_vector.push_back(mid_info(pid,dis));
			//circle(img, pid, 2, Scalar(0,0,255), CV_FILLED, 8);	
		}	


	}

	std::sort(pmids_vector.begin(), pmids_vector.end(),disgreater);
	for(size_t i = 0; i < pmids_vector.size(); i++)
	{
		pmids_vector[i].index = i+1;
	}
	pmids_vector.insert(pmids_vector.begin(), mid_info(start,0,0));
	int goalind = pmids_vector.size();
	pmids_vector.insert(pmids_vector.end(), mid_info(goal,0,goalind));
	
	
	/*for(auto i:pmids_vector)
	{
		cout <<i.index<<" "<< i.dis <<" " << i.pos.x << " " << i.pos.y<< endl; 
	}*/
	return true;

}
bool Env2d::GenerateGaps_dynamic(Point2d current)
{
	
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
	
	cv::imwrite("./result/1_with_midslen_angle.png", img);
	
}

void Env2d::draw_traj()
{
	for(auto it=initTraj.begin(); it!=(initTraj.end()-1); ++it)
	{
		//const mid_info& mid = pids_que.top(); 
		Point2d ele_pos = *it;
		cout<<"&&&&&&&&&&&&&"<<ele_pos<<endl;
		draw_circle(ele_pos,2,Scalar(255,0,0));
		draw_line(*it, *(it+1));
		//cout << it->x << " " << it->y << endl;
		//pids_que.pop();
		
	}
	Point2d& ele_pos = initTraj.back();
	draw_circle(ele_pos,2,Scalar(255,0,0));
	cv::imwrite("../result/2d/3_inittraj.png", img);
	
}
void Env2d::draw_interptraj()
{
	for(auto it=interpTraj.begin(); it!=interpTraj.end()-1; ++it)
	{
		//const mid_info& mid = pids_que.top(); 
		Point2d& ele_pos = *it;
		draw_circle(ele_pos,2,Scalar(255,0,0));
		draw_line(*it, *(it+1));
		//cout << it->x << " " << it->y << endl;
		//pids_que.pop();
		
	}
	Point2d& ele_pos = interpTraj.back();
	draw_circle(ele_pos,2,Scalar(255,0,0));
	cv::imwrite("../result/2d/3_interptraj.png", img);
	
}
bool Env2d::isPosinObs(const Point2d& point)
{
	for(size_t i = 0; i < obs_num; i++)
	{

		// point is in the circle
		if(norm(point-centers[i]) < radius[i])
			return true;
	}
	return false;
}
bool Env2d::checkSegment(const Point2d& start, const Point2d& point)
{
	//cout << "*****is pos free : "<< point.x << " " << point.y << endl;

	double dis_cu2p(norm(point - start));
	Point2d dir = (point - start) * (1/dis_cu2p); 
	for(size_t i = 0; i < obs_num; i++)
	{

		// point is in the circle
		if(norm(point-centers[i]) < radius[i])
			return false;

		// compute the euclidean distance between A and B
		
		//cout <<"***********"<<i << "th obs: " <<centers[i].x <<" " << centers[i].y<<endl;
		if (dis_cu2p<eps)
			continue;
		// compute the direction vector D from A to B
		//???
		//cout << dir.x <<" "<< dir.y << endl;
		Point2d cur2obs = centers[i] - start ;
		// the projection 
		double t = cur2obs.dot(dir);
		double tmp = cur2obs.dot(point - start)/dis_cu2p;
		//cout << "cur2obs"<< cur2obs.x << " " << cur2obs.y<< endl;
		//cout << "dir"<< dir.x << " " << dir.y<< endl;
		//cout << "t: " << t <<" " << tmp<< endl;
		//cout << "dis_cu2p: " << dis_cu2p << endl;
		
		if(t<0) continue;
		if(t>dis_cu2p) continue;
		// the distance between obs and line
		Point2d closest_point(t*dir);
		//cout <<"the clos point is : "<< closest_point.x <<" "<< closest_point.y << endl;
		if(norm(cur2obs)*norm(cur2obs) - t*t <= radius[i]*radius[i]) 
			return false;

	}
	//cout << "is free!!" << endl;
	return true;
}
bool Env2d::isPosFree(const Point2d& point)
{
	//cout << "*****is pos free : "<< point.x << " " << point.y << endl;

	double dis_cu2p(norm(point - currentpos));
	Point2d dir = (point - currentpos) * (1/dis_cu2p); 
	for(size_t i = 0; i < obs_num; i++)
	{

		// point is in the circle
		if(norm(point-centers[i]) < radius[i])
			return false;

		// compute the euclidean distance between A and B
		
		//cout <<"***********"<<i << "th obs: " <<centers[i].x <<" " << centers[i].y<<endl;
		if (dis_cu2p<eps)
			continue;
		// compute the direction vector D from A to B
		//???
		//cout << dir.x <<" "<< dir.y << endl;
		Point2d cur2obs = centers[i] - currentpos ;
		// the projection 
		double t = cur2obs.dot(dir);
		double tmp = cur2obs.dot(point - currentpos)/dis_cu2p;
		//cout << "cur2obs"<< cur2obs.x << " " << cur2obs.y<< endl;
		//cout << "dir"<< dir.x << " " << dir.y<< endl;
		//cout << "t: " << t <<" " << tmp<< endl;
		//cout << "dis_cu2p: " << dis_cu2p << endl;
		
		if(t<0) continue;
		if(t>dis_cu2p) continue;
		// the distance between obs and line
		Point2d closest_point(t*dir);
		//cout <<"the clos point is : "<< closest_point.x <<" "<< closest_point.y << endl;
		if(norm(cur2obs)*norm(cur2obs) - t*t <= radius[i]*radius[i]) 
			return false;

	}
	//cout << "is free!!" << endl;
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
	
	//if(norm(point-currentpos)>30)
		//return false;
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
	int numsize = initTraj.size();
	if(numsize>1)
	{
		Point2d last = initTraj[numsize-1];
		Point2d lasttwo = initTraj[numsize-2];
		Point2d dir = last-lasttwo;
		Point2d newdir = point-last;
		//if(dir.dot(newdir)<0)
		if(norm(point-currentpos)>30)
		{
			cout <<"add !!!!!!!!!!!!!!!!!!!!!!"<<endl;
			Point2d newp((currentpos*0.3+point*0.7));
			initTraj.push_back(newp);
			newp = Point2d(currentpos*0.7+point*0.3);
			initTraj.push_back(newp);
		}
			

	}
	cout << " *********Add a mid point *****" << endl;
	cout << point.x << " " << point.y << endl;
	initTraj.push_back(point);	
	currentpos = point;
	cur2goal_dis = norm(currentpos - goal);
	cout << currentpos <<" "<< cur2goal_dis <<endl ;

}

void Env2d::changecoord(vector<Point2d>& init, vector<Point2d>& result)
{
	result.clear();
	double len, angle;
	for(auto it=init.begin(); it!=init.end(); it++)
	{

		len = norm(*it);
		if(it->x < 0.0001) 
			angle = 0;
		else
			angle = std::atan(it->y/it->x);
		Point2d newp(len,angle);
		result.push_back(newp);
		//cout << "len and angle "<<len<<" "<<angle<<endl; 
	}


}
void Env2d::changeback(vector<Point2d>& init, vector<Point2d>& result)
{
	result.clear();
	//cout <<"back begin!"<<endl;
	cout << init.size()<<endl;
	for(auto it=init.begin(); it!=init.end(); it++)
	{
		double xx = it->x*cos(it->y);
		double yy = it->x*sin(it->y);
		Point2d newp(xx,yy);
		//cout << "back : " << xx << " " << yy<<endl;
		result.push_back(newp);
	}

}
bool Env2d::plan()
{
	initTraj.push_back(start);
	while(currentpos != goal)
	{
		cout << currentpos<<endl;
		if(TryGoal())
		{
			cout << "currentpos: " << currentpos <<endl;
			cout << "goal is reaachable!!!" << endl;
			//AddMidPoit((goal*0.3+currentpos*0.7));
			//AddMidPoit((goal*0.7+currentpos*0.7));
			//AddMidPoit((goal*0.5+currentpos*0.5));
			AddMidPoit(goal);
			
			return true;
		}
		bool foundonemid = false;
		
		for(auto it = pmids_vector.begin(); it != pmids_vector.end(); ++it)
		{
			const Point2d p_mid= it->pos;
			if(norm(p_mid-currentpos)<3) 
				continue;
			if(isReachable(p_mid))
			{
				foundonemid = true;
				cout << "currentpos: " << currentpos <<endl;
				//AddMidPoit((p_mid+currentpos)*0.5);
				//AddMidPoit((p_mid*0.3+currentpos*0.7));
			 //AddMidPoit((p_mid*0.7+currentpos*0.7));
				AddMidPoit(p_mid);
				//pmids_vector.erase(it);
				
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



bool Env2d::cubic_interp_seveal(vector<Point2d>& uninterp)
{
	cout <<"unisize:" <<uninterp.size()<<endl;
	//size_t pos_len = 0;
	if(!(uninterp.size()>1))
		return false;

	
	//interpTraj.clear();
	
	std::vector<double> X,Y;
	std::map<double, double> points;
	for(auto it = uninterp.begin(); it!=uninterp.end(); it++)
	{
		points[it->x] = it->y;
		
	}
	//std::sort(points.begin(),points.end(),Sort_map);
	for(std::map<double,double>::iterator it = points.begin(); it != points.end(); ++it) 
	{
  		X.push_back(it->first);
  		cout << it->first << endl;
  		Y.push_back(it->second);
	}

	

	tk::spline s;
	s.set_points(X,Y);

	int interpnum = abs(X.front()-X.back())/3;
	double stepsize = abs(goal.x - start.x)/interpnum;
	double val = X.front();
	for(;val<=X.back();val=val+3)
	{
			
			interpTrajlen_ang.push_back(Point2d(val, s(val)));
	}
	if(val!=X.back()+3)
		interpTrajlen_ang.push_back(Point2d(X.back(), s(X.back())));
	//Point2d lastone = Point2d(X.back().x, Y.back().y);
	//interpTraj.push_back(lastone);
	//cout << interpTrajlen_ang.size()<<endl;
	cout << " success!" <<endl;
	return true;
}

bool Env2d::cubic_interp(int groupsize)
{
	interpTrajlen_ang.clear();
		
	auto it = initlen_ang.begin();
	for(; it<initlen_ang.end()-groupsize; it=it+groupsize-1)
	{
		std::vector<Point2d> onegroup(it,it+groupsize);
		cout << onegroup.size()<<endl;
		if(! cubic_interp_seveal(onegroup))
			return false;
	}
	std::vector<Point2d> lastgroup(it,initlen_ang.end());
	if(!cubic_interp_seveal(lastgroup))
		return false;
	//cout << interpTrajlen_ang.size()<<endl;
	return true;
}
void printvpoint(const Point2d & point)
{
	std::cout << point.x <<" "<< point.y<<std::endl;

}
bool Env2d::draw_mid_traj(const std::vector<int> &traj)
{
	if(traj.size()<2)
	{
		std::cout <<" draw_mid_traj problem! "<< traj.size() << "elements"<<std::endl;
		return false;
	}
	for(size_t i =0; i < traj.size()-1; i++)
	{
		//const mid_info& mid = pids_que.top(); 
		//Point2d ele_pos = *it;
		//cout<<"&&&&&&&&&&&&&"<<ele_pos<<endl;
		//draw_circle(ele_pos,2,Scalar(255,0,0));
		//std::cout << pmids_vector[i].index << pmids_vector[i+1].index << std::endl;
		//printvpoint(pmids_vector[i].pos);
		//printvpoint(pmids_vector[i+1].pos);

		draw_line(pmids_vector[traj[i]].pos, pmids_vector[traj[i+1]].pos);
		//cout << it->x << " " << it->y << endl;
		//pids_que.pop();
		
	}
	return true;
}
void printintvector(std::vector<std::vector<int> > vec)
{
	for(size_t i =0; i< vec.size(); i++)
	{
		std::cout<<i<<" th: ";
		for(size_t j = 0; j < vec[i].size(); j++)
		{
			std::cout << vec[i][j]<<" ";
		}
		std::cout << std::endl;
	}
}
void printintvector(std::vector<int> vec)
{
	for(size_t i =0; i< vec.size(); i++)
	{
		if(vec[i] != 0)
		std::cout<<'['<<i<<"]: " << vec[i] <<" ";
			
	}
	std::cout << std::endl;
}
bool Env2d::plantraj(int nodenum, std::vector<int> traj, std::vector<std::vector<int> > searchnode_rec)
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
		searchnode = searchnode_rec;

		draw_mid_traj(traj);
		printintvector(searchnode_rec);
		return true;
	}
	else
	{
		if(searchnode_rec.size()+1 < traj[nodenum])
		{
			std::cout<<"fail!! searchnode number" << searchnode_rec.size()<<" " <<traj[nodenum] <<std::endl;
			return false;
		}
		std::vector<int> &cur_node = searchnode_rec[traj[nodenum]];
		int node_len = cur_node.size();
		if(node_len<2 || cur_node[0] >= node_len-1)
		{
			// this node fails
			std::cout << "back!!!!" << endl;
			traj.pop_back();
			cur_node[0] = 0;
			return plantraj(nodenum-1,traj,searchnode_rec);
		}
		else
		{
			if(nodenum==0)
			{ // start point

				for(size_t i = cur_node[0]+1; i< node_len; i++)
				{
					cur_node[0] = i;
					cout <<" search in node "<<traj[nodenum] << std::endl;
					//Point2d pos = pmids_vector[cur_node[i]].pos;
					//if(norm(pos-goal) < norm(curpos - goal))
					
						
						if(cur_node[i] != pmids_vector[cur_node[i]].index)
						{
							std::cout <<"problem!!! 654 line "<<std::endl;
							return false;
						}
						Point2d pos = pmids_vector[cur_node[i]].pos;
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
				Point2d curpos = pmids_vector[traj[nodenum]].pos;
				bool foundone = false;
				for(size_t i = cur_node[0]+1; i< node_len; i++)
				{
					cur_node[0] = i;

					cout <<" search in node " << traj[nodenum] <<" "<< i<< std::endl;
					Point2d pos = pmids_vector[cur_node[i]].pos;
					if(norm(pos-goal) < norm(curpos - goal))
					{
						//printvpoint(curpos);
						//printvpoint(pos);
						//std::cout <<"compare distantce"<< norm(pos-goal) << " "<< norm(curpos-goal) << std::endl;
						if(cur_node[i] != pmids_vector[cur_node[i]].index)
						{
							std::cout <<"problem!!! 654 line "<<std::endl;
							return false;
						}
						if(improved)
						{
							if(nodenum>1)
							{
								Point2d backpos = pmids_vector[traj[nodenum-1]].pos;
								if(connected[traj[nodenum-1]][cur_node[i]] && norm(backpos-curpos) + norm(curpos-pos) > norm(backpos - pos))
								{
									cout << "improved!!!! nodenm: "<< nodenum << endl;
									nodenum = nodenum-1;
									traj.pop_back();

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
					std::cout<<"no found back!"<<std::endl;
					traj.pop_back();
					cur_node[0] = 0;
					return plantraj(nodenum-1,traj,searchnode_rec);
				}
			}
			
		}

	}
	std::cout <<"plantraj end???" <<std::endl;
	return true;
}

bool Env2d::plantraj_v2(int nodenum, std::vector<int> traj, std::vector<int> searchnode_rec)
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

		draw_mid_traj(traj);
		printintvector(searchnode_pointer_cur);
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
					searchnode_rec[traj[nodenum]] = i;
					cout <<" search in node "<<traj[nodenum] << std::endl;
					//Point2d pos = pmids_vector[cur_node[i]].pos;
					//if(norm(pos-goal) < norm(curpos - goal))
					
						
						if(cur_node[i] != pmids_vector[cur_node[i]].index)
						{
							std::cout <<"problem!!! 654 line "<<std::endl;
							return false;
						}
						Point2d pos = pmids_vector[cur_node[i]].pos;
						//printvpoint(start);
						//printvpoint(pos);

						traj.push_back(cur_node[i]);
						for(int k =0; k < traj.size(); k++)
						{
							std::cout << traj[k] << " ";
						}
						std::cout << std::endl;
						return plantraj_v2(nodenum+1,traj,searchnode_rec);
							
					
				}
			}
			else 
			{
				Point2d curpos = pmids_vector[traj[nodenum]].pos;
				bool foundone = false;
				for(size_t i = searchnode_rec[traj[nodenum]]+1; i< node_len; i++)
				{
					searchnode_rec[traj[nodenum]] = i;

					cout <<" search in node " << traj[nodenum] <<" "<< i<< std::endl;
					Point2d pos = pmids_vector[cur_node[i]].pos;
					if(norm(pos-goal) < norm(curpos - goal))
					{
						//printvpoint(curpos);
						//printvpoint(pos);
						//std::cout <<"compare distantce"<< norm(pos-goal) << " "<< norm(curpos-goal) << std::endl;
						if(cur_node[i] != pmids_vector[cur_node[i]].index)
						{
							std::cout <<"problem!!! 654 line "<<std::endl;
							return false;
						}
						if(improved)
						{
							while(nodenum>1)
							{
								cout << nodenum<<endl;
								Point2d backpos = pmids_vector[traj[nodenum-1]].pos;
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
						return plantraj_v2(nodenum+1,traj,searchnode_rec);
							
					}
				}
				if(!foundone)
				{
					std::cout<<"no found back!"<<std::endl;
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

void Env2d::replan(int replan_startpoint, std::vector<int> oldtraj,std::vector<std::vector<int> > searchnode_rec)
{
	//int nodeind = oldtraj[replan_startpoint+1];
	for(size_t i = replan_startpoint+1; i < oldtraj.size(); i++)
	{
		searchnode_rec[oldtraj[i]][0] = 0;
	}
	oldtraj.erase(oldtraj.begin()+replan_startpoint+1, oldtraj.end());
	plantraj(replan_startpoint,oldtraj,searchnode_rec);

}

void Env2d::replan_v2(int replan_startpoint, std::vector<int> oldtraj,std::vector<int> searchnode_rec)
{
	//int nodeind = oldtraj[replan_startpoint+1];
	for(size_t i = replan_startpoint+1; i < oldtraj.size(); i++)
	{
		searchnode_rec[oldtraj[i]] = 0;
	}
	oldtraj.erase(oldtraj.begin()+replan_startpoint+1, oldtraj.end());
	plantraj_v2(replan_startpoint,oldtraj,searchnode_rec);

}

int main()
{
	Env2d env(300,300,0,0,300,300,1);
	env.GenerateGaps();
	
	env.setconnet();
	
	/*std::vector<int> &tmp = env.searchnode[9];
	tmp.erase(tmp.begin()+2, tmp.end());
	std::vector<int> &tmp2 = env.searchnode[16];
	tmp2.erase(tmp2.begin()+2, tmp2.end());
	//env.draw_midpoints();
	*/

	std::vector<int> traj_test;
	traj_test.push_back(0);
	std::vector<int> searchnode_rec;
	searchnode_rec = env.searchnode_pointer_cur;

	env.plantraj_v2(0,traj_test,searchnode_rec);
	printintvector(env.traj_mid_index);
	//printintvector(env.searchnode);
	//std::cout << env.traj_mid_index.size()<<std::endl;
	env.replan_v2(3,env.traj_mid_index,env.searchnode_pointer_cur);
	printintvector(env.traj_mid_index);
	env.replan_v2(1,env.traj_mid_index,env.searchnode_pointer_cur);
	printintvector(env.traj_mid_index);
	env.replan_v2(1,env.traj_mid_index,env.searchnode_pointer_cur);
	printintvector(env.traj_mid_index);
	env.replan_v2(2,env.traj_mid_index,env.searchnode_pointer_cur);
	printintvector(env.traj_mid_index);
	//printintvector(env.searchnode);

	traj_test = env.traj_mid_index;
	//env.replan(3,traj_test,env.searchnode);

	
	//env.plantraj(2,traj_test);
	//traj_test = env.traj_mid_index;
	//env.plantraj(2,traj_test);
	env.draw_midpoints();
	/*
	env.plan();
	env.draw_traj();
	env.changecoord(env.initTraj, env.initlen_ang);
	
	//env.interpTraj.clear();
	bool suc = env.cubic_interp(20);
	env.changeback(env.interpTrajlen_ang, env.interpTraj);
	cout << suc << endl;
	
	env.draw_interptraj();
	*/
	
	//env.draw_midpoints();
	return 0;

}