#include <iostream>
using namespace std;

int main(){
    int* p = new int;  //allocate space for 1 int on the loop
    
    *p = 123; // write a value into that space
    cout << "The value at p= " << *p << "\n";
    
    delete p; // free the memory when done
    
    return 0;
}