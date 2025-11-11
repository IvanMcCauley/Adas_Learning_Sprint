#include <iostream>
#include <chrono>
#include "braking/braking.hpp"

int main(){
    using clock = std::chrono::high_resolution_clock;

    const int iters = 1'000'000; //1e6
    double sum = 0.0; //accumulate to keep work "observable"

    //Simple sweep: vary speed a bit; keep other params fixed
    double reaction_time = 1.5; // s
    double decel = 8.0; // m/s^2
    double distance = 60.0; //m
    double margin = 0.10;

    auto t0 = clock::now();
    for (int i =0; i < iters; ++i){
        double speed = (i % 51); // 0..50m/s
        // Call the core logic, accumulate something so the compiler can't remove it
        bool b = needs_brake(distance, speed, reaction_time, decel, margin);
        // Also do the math pieces to approximate typical usage
        sum += reaction_distance(speed, reaction_time)
             + compute_brake_distance(speed, decel)
             + (b ? 1.0 : 0.0);
    }
    auto t1 = clock::now();

    //Prevent optimizing away 'sum'
    volatile double sink = sum;
    (void)sink;

    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    double ns_per_iter = (micros * 1000.0) / iters;

    std::cout << "Iterations: " << iters << "\n";
    std::cout << "Total time: " << micros << "us\n";
    std::cout << "Avg: " << ns_per_iter << "ns/iter\n";
    return 0;
}
