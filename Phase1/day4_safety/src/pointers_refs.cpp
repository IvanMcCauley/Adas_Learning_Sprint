#include <iostream>
using namespace std;

void increment_ptr(int* v) { if (v) (*v)++; }


int main(){
    int x = 42;  //normal int in memory
    cout << "x = " << x << "\n"; // prints value stored in x
    cout << "Address of x = " << &x << "\n"; // prints adress of x

    int* p = &x; //p points to x (stores x's address)
    cout << "p = " << p << "\n"; // prints address stored in p
    cout << "*p = " << *p << "\n"; //value at that address

    *p = 99;
    cout << "after *p=99:  x=" << x << "  *p=" << *p << "\n";
    
    increment_ptr(&x);
    cout << "after increment_ptr(&x): x=" << x << "\n";
    return 0;


}