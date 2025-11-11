#include <iostream>
#include <vector>
using namespace std;

vector<int> make_dataset(int N){
    vector<int> v;
    v.reserve(N);
    for (int i = 0; i < N; ++i) v.push_back(i * i); //fill with squares
    return v; //safe: copies/moves the vector to the caller
}

int main(){
    auto data = make_dataset(6);
    cout << "size=" << data.size() << " elem[3]=" << data[3] << "\n";
    
}