#include <iostream>
using namespace std;

int main(){
    int arr[5];
    cout << "Enter 5 numbers:\n";
    for (int i=0; i<5; ++i){
        cin >> arr[i];
    }
    int sum = 0;
    int maxv = arr[0];
    for (int i = 0; i<5; ++i){
        sum += arr[i];
        if (arr[i] > maxv) maxv = arr[i];
    }    
    cout << "Sum = " << sum << "\n";
    cout << "Max: " << maxv << "\n";
    
    return 0;

}    
