#include <vector>
#include <iostream>

int main()
{
	std::vector<int> v={1,2,3,4,5,6,7};
	v.erase(v.begin()+2);
	for(auto i:v)
		std::cout<<i<<std::endl;
}