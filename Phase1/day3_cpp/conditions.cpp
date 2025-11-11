#include <iostream>
using namespace std;

int main(){
    cout << "Enter an integer: ";
    int x;
    cin >> x;

    if (x > 0) {
        cout << "X is positive\n";
    } else if (x < 0) {
        cout << "X is negative\n";
    } else { 
        cout << "X is 0\n";
    }    
    
    if (x % 2 == 0){
        cout << "X is even\n";
    } else {
        cout << "X is odd\n";
    }
    return 0;
}