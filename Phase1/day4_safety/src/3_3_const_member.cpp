#include <iostream>
using namespace std;

struct Counter {
    int n{0}; //member variable, starts at 0

    void inc() { n++; } //normal member function, can modify n

    int value()const{ //const member function
        //n++ // would not compile as cant modify n
        return n;//read only access is fine
    }

};

int main(){
    Counter c;
    c.inc();  //n=1
    cout << c.value() << "\n"; //prints 1
}