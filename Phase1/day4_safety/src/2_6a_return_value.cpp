#include <iostream>
using namespace std;
int make_value(){
    int x = 42;
    return x;
}

int main(){
    int v = make_value();
    cout << "v = " << v << "\n";

}