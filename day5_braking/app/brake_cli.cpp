#include <iostream>
#include <iomanip>
#include "braking/braking.hpp"

// Simple interactive CLI for braking calculations (SI units)
int main(){
    // Initializaiions
    double speed;            // m/s
    double reaction_time;    // s
    double decel;            // m/s^2 (positive)
    double distance_to_obs;  //m


    // Inputs
    std::cout << "Enter speed (m/s): ";
    if (!(std::cin >> speed)) return 1;                 // speed input

    std::cout << "Enter reaction time (s): ";
    if (!(std::cin >> reaction_time)) return 1;       // reaction time input

    std::cout << "Enter decel magnitude (m/s^2): ";
    if (!(std::cin >> decel)) return 1;                 // decel input

    std:: cout << "Enter distance to obstacle (m): ";
    if (!(std::cin >> distance_to_obs)) return 1;       // distance to obstacle input


    // Calculations using the above inputs and predefined functions
    double d_react = reaction_distance(speed, reaction_time);
    double d_brake = compute_brake_distance(speed, decel);
    double d_total = d_react + d_brake;

    double margin = 0.10; // 10% policy buffer
    bool brake = needs_brake(distance_to_obs, speed, reaction_time, decel, margin);
    double d_total_with_margin = d_total * (1.0 + margin);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nReaction distance:            " << d_react  << " m\n";
    std::cout << "Brake distance:               " << d_brake   << " m\n";
    std::cout << "Total distance (no margin):   " << d_total   << " m\n";
    std::cout << "Total distance (+10% margin): " << d_total_with_margin << " m\n";
    std::cout << "Decision (margin=10%):        " << (brake ? "BRAKE" : "OK") << "\n";

    return 0;
}