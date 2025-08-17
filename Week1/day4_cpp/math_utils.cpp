#include "math_utils.h"

long long factorial (int n) {
    long long res = 1;
    for (int t = 2; t <= n; ++t){
        res *= t;
    }
    return res; // n=0 or 1 -> res stays 1
}

long long sum_to(int n) {
    if (n <= 0) return 0;
    long long s = 0;
    for (int t =1; t <= n; ++t){
        s += t;
    }
    return s;

}