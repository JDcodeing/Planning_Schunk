#include <vector>
#include <iostream>
#include <fstream>
//#include <stdlib.h>


bool loadObs(const std::string filename, bool displayObsInfo)
{

	std::ifstream obsfile;
	obsfile.open(filename.c_str(), std::ifstream::in);
	int num = 0;
	if(obsfile.is_open())
	{
		std::string line;
		
		while(getline(obsfile, line))
		{
			std::cout << line;
			
		}
		obsfile.close();
		

	
		
		return true;

	}
	else
	{
		 std::cerr << "There was a problem opening the input file!\n";
		 return false;
	}
  
}
int main()
{
	
  //return 0;
	//loadObs("/home/juan-robotics/moveit_juan/src/planning_test/obs.scene",true);
	//loadObs("/home/juan-robotics/moveit_juan/src/planning_test/src/obs.scene",true);
	  loadObs("/home/juan-robotics/moveit_juan/src/planning_test/src/obs.scene",true);

  	return 0;


}