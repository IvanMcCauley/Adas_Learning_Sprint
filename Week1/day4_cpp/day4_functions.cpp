#include <iostream>
#include "math_utils.h"
using namespace std;

int main(){
    cout << "n? ";
    int n;
    cin >> n;

    cout << "factorial(" << n << ") = " << factorial(n) << "\n";
    cout << "sum_to(" << n << ") = " << sum_to(n) << "\n";
    return 0;

}