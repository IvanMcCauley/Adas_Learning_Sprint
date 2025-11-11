#include <iostream>
#include <vector>
using namespace std;

int main(){
    int N;
    cout << "How many samples? ";
    if (!(cin >> N) || N < 0) return 0;

    //heap-backed dynamic storage managed by vector
    vector<int> samples;
    samples.reserve(N); //optional perf hint

    for (int i = 0; i < N; ++i) samples.push_back(i * 10);
    
    cout << "size=" << samples.size() << " last=" << (N? samples.back() : -1) << "\n";
    

}