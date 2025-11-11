#include <iostream> 
using namespace std;

// return reference to element i in the array
int& element_at(int arr[], int i){
    return arr[i]; //reference to arr[i]
}

int main(){
    int arr[3] = {10, 20, 30};

    cout << "Before: arr[1] = " << arr[1] << "\n";

    //modify element via return-by-reference
    element_at(arr, 1) = 99;

    cout << "After: arr[1] = " << arr[1] << "\n";
    return 0;

}
