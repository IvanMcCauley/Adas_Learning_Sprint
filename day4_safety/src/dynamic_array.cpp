#include <iostream>
using namespace std;

int main (){
    int N = 5; // used for the array size

    // allocate array of N ints on the heap
    int* arr = new int[N];

    //fill the array
    for (int i=0; i < N; ++i){
        arr[i] = i * 10;
    }
    
    for (int i=0; i<N; ++i){
        cout << "arr[" << i << "] = " << arr[i] << "\n";
    }

    //free the memory
    delete[] arr;

    return 0;
    
}