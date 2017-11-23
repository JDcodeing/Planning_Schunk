#include <Eigen/Geometry>
#include <iostream>

int main()
{
	Eigen::Vector3d point1(Eigen::Vector3d(0,0,0));
	Eigen::Vector3d point2(Eigen::Vector3d(2,3,5));
	Eigen::Vector3d dir_cu2p = (point2 - point1).normalized();
	
	Eigen::Affine3d transformation= Eigen::Affine3d::Identity();
	transformation.translation() = point2;
	std::cout << transformation.linear()<< std::endl;

	std::cout << transformation.translation()<< std::endl;
	return 0;
}