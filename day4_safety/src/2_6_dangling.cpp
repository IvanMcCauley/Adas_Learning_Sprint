#include <iostream>
using namespace std;

int* make_pointer(){
    int x = 42; //local variable on the stack
    return &x; //returns address of something that goes out of scope
}

int main(){
    int* p = make_pointer();
    cout << "p points to " << p << "\n";
    cout << "*p (dangling) = " << *p << "\n";//undefined behaviour
}