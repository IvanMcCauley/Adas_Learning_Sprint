#include <iostream> 
using namespace std; 

int main(){
    int*p = new int(7);
    cout << "*p = " << *p << "\n";
    delete p; //first delete (OK)
    delete p;

}