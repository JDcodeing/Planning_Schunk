// lower_bound/upper_bound example
#include <iostream>     // std::cout
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>       // std::vector
#include <string>
#include <sstream>
using namespace std;

int main (int argc, char **argv) {
	std::vector<int> a,b,c;
	for(int i = 0; i < 6; i++) 
	{
		b.push_back(i*2);
		a.push_back(i);
	}
	double ha = std::abs(b[1]-a[1]);
	auto it = std::max_element(a.begin(),a.end());
	cout << *it << endl;
	cout << ha << endl;
  return 0;
}