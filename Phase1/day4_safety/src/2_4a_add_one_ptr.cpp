#include <iostream>
using namespace std;

void add_one_ptr(int* v){
    if (v) (*v)++; //check not null, then increment

}

int main(){
    int x = 5;
    add_one_ptr(&x); //pass address of x
    cout << "after pointer fn: x = " << x << "\n";
}
