// lower_bound/upper_bound example
#include <iostream>     // std::cout
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>       // std::vector
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;

int main (int argc, char **argv) {
	int a;
    if(argc==2){
        a=argv[1];
        b=argv[2];
    }
    else{
        cout << "Please enter a number:";
        cin >> a;
        cout << "Please enter another number:";
        cin >> b;
    }
    cout << "Addition:" << a+b << endl;
    cout << "Subtaction:" << a-b << endl;
    cout << "Multiplycation:" << a*b << endl;
    cout << "Division:" << static_cast<long double>(a)/b << endl;
    system("pause");
    return 0;

  return 0;
}