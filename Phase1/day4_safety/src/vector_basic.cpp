#include <iostream>
#include <vector>
using namespace std;

int main(){
    int N = 5;
    vector<int> a; //empty
    a.reserve(N); //pre-allocate
    for (int i=0; i<N; ++i){
        a.push_back(i*10);
    }

    for (int i = 0; i < (int)a.size(); i++){
        cout << "a[" << i << "] = " << a[i] << "\n";
    }
}


