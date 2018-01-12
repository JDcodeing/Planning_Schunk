// lower_bound/upper_bound example
#include <iostream>     // std::cout
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>       // std::vector
#include <string>
#include <fstream>
#include <cmath>
#include "spline.h"

using namespace std;
/*
void interpolate(const Eigen::Vector3d &from, const Eigen::Vector3d &to, const double t, Eigen::Vector3d &state) 
{
    state = from + (to-from)*t;
}

void interpplateCartesianPath(std::vector<Eigen::Vector3d> &CartesianPath, const double stepsize_cart)
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
double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

Eigen::Vector3d Vector3dRand(Eigen::Vector3d vec, double min, double max)
{
  Eigen::Vector3d res;
  for(int i = 0; i < 3; i++)
  {
    res[i] = vec[i] + fRand(min,max);
  }
  return res;
}
*/
void print(vector<double> a)
{
  //for(int i = 0; i < a.size(); i++)
  for(auto i:a)
    cout << i<<" ";
  cout << endl;
  return;
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

void testpointer(double& a)
{
  std::cout << a << std::endl;
  a = 4;

}

void printfile(ofstream& myfile)
{
  myfile << "This is the first cell,, in the first column.\n";
      myfile << "a,b,c\n";
      myfile << "c,s,v\n";
      myfile << "1,2,3.456\n";
      myfile << "semi,colon";
}

int main (int argc, char **argv) {

	double aa[] ={1.2,1.4,1.6};

  double ab[] ={-1.2,1.4};
  double ac[] ={-2.2,1.8,2.1};
  vector<double> a(aa,aa+3),b(ab,ab+2),c(ac,ac+3);

  
  vector<vector<double> > vecvec,res;
  vecvec.push_back(a);
  vecvec.push_back(b);
  //vecvec.push_back(c);
  /*cubic_interp(res,vecvec,0.2);
  for(int i = 0; i < res.size() ; i ++)
  {
    for(int j = 0; j < 3; j++)
    {
      cout<< res[i][j]<<" ";
    }
    cout << endl;
  }*/
  //a.swap(b);
  print(a);

    double doublec = 10.2;
    double &ref = doublec;
    testpointer(doublec);
    std::cout << doublec << std::endl;



      ofstream myfile;
      string name = "result/example" + std::to_string(2) + ".csv";
      myfile.open (name);
      printfile(myfile);
      myfile.close();
      

  return 0;
}