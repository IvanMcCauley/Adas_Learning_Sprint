#include <iostream>
using namespace std;

void add_one_ref(int& v){
    v++;
}

int main(){
    int y = 5;
    add_one_ref(y);
    cout << "after ref fn: y = " << y << "\n";
    
}