#include <iostream>
#include <vector>
using namespace std;

//function takes a const reference to a vector of ints
int sum_const_ref(const vector<int>& v){
    int s = 0;
    for (int x : v) s +=x ;
    //v[0] = 99; // this would not compile as v is a const
    return s;
}

int main (){
    vector<int> a = {1,2,3,4};
    cout << "sum = " << sum_const_ref(a) << "\n"; // should print 10
}