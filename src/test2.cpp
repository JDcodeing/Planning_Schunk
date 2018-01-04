// lower_bound/upper_bound example
#include <iostream>     // std::cout
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>       // std::vector
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
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

int main (int argc, char **argv) {
	std::vector<Eigen::Vector3d> path;
	Eigen::Vector3d v(1,3,6);
	Eigen::Vector3d res = Vector3dRand(v,-0.02,0.02);
	cout << res;

  return 0;
}