// C++ program to illustrate runtime
// error by un-assigned variables
#include <iostream>
using namespace std;
  
// Driver Code
int main()
{
    int N = 3;
  
    int arr[N];
  
    for (int i = 0; i < N; i++) {
        arr[i] = i;
    }
  
    for (int i = 0; i < N; i++) {
        cout << arr[i] << " ";
    }
  
    return 0;
}















