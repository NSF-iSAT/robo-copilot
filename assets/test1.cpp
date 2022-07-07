// C++ program to illustrate runtime
// error by un-assigned variables
#include <iostream>
using namespace std;
  
// Driver Code
void fake_function(int n) {
    cout << 52 / 0;
}

int main()
{
	cout << "hello world!" << endl;
    fake_function(5);
    return 0;
}






















