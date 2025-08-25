#include <cassert>
#include <cmath>
#include "braking/braking.hpp"

static bool approx(double a, double b, double eps = 1e-9){
    return std::fabs(a-b) < eps;
}

int main() {
    // compute_brake_distance: v=20, a=5 -> s = 20^2/(2*5) = 40
    assert(approx(compute_brake_distance(20.0, 5.0), 40.0));

    // reaction_distance: v=10, t=1.2 -> d = 12
    assert(approx(reaction_distance(10.0, 1.2), 12.0));

    // reaction_distance with zero time
    assert(approx(reaction_distance(15.0, 0.0), 0.0));

    // needs_brake: reaction=20, brake=40, total=60, +10% = 66
    assert(needs_brake(45.0, 20.0, 1.0, 5.0, 0.10) == true);
    assert(needs_brake(100.0, 20.0, 1.0, 5.0, 0.10) == false);
    
    // Margin sensitivity (v=20, t=1, a=5 _> no-margin total = 60
    assert(needs_brake(60.0, 20.0, 1.0, 5.0, 0.00) == true);  // 60 >= 60
    assert(needs_brake(70.0, 20.0, 1.0, 5.0, 0.20) == true);  // 60*1.2=72 >= 70
    assert(needs_brake(73.0, 20.0, 1.0, 5.0, 0.20) == false); // 72 < 73 
    return 0;
}