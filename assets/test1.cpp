// C++ program to illustrate runtime
// error by un-assigned variables
#include <iostream>
using namespace std;
  
// Driver Code
int main()
{
    long long N;
	cout << "hello world!" << endl;
  
    // N is assigned garbage value
    long arr[N];
  
    cin >> N;
  
    for (int i = 0; i < N; i++) {
        cin >> arr[i];
    }
  
    for (int i = 0; i < N; i++) {
        cout << arr[i] << " ";
    }
  
    return 0;
}












