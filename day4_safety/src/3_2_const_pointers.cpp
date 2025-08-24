#include <iostream>
using namespace std;

int main(){
    int x = 5, y = 9; 

    const int* p1 = &x; //pointer to const int -> *p1 cant change, p1 can point elsewhere
    // *p1 =7  // not allowed
    p1 = &y; // allowed, pointer moved

    int* const p2 = &x; // const pointer to int -> pointer fixrd, *p2 can change
    *p2 = 7; // allowed (x becomes 7)
    // p2 = &y // not allowed, cant repoint

    const int* const p3 = &x; // const pointer to a const int -> neither can change
    (void)p3;
    (void)p1; //silence unsused warning

    cout << "x=" << x << ", y=" << y << "\n";
}