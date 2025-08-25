#include <iostream>
#include <iomanip>
#include <string>  //for std::string
#include <cstdlib> //for std::atof
#include "braking/braking.hpp"

// Simple interactive CLI for braking calculations (SI units)
int main(int argc, char* argv[]) {
    
    // Default safety margin = 10%
    // If user runs: ./brake_cli --margin 0.2 --> margin becomes 0.2 (20%)
    double margin = 0.10;
    if (argc >= 3 && std::string(argv[1]) == "--margin") {
        margin = std::atof(argv[2]); // turn text like "0.2" into 0.2
        if (margin < 0.0) { //simple guard: no negatives
            std::cerr << "Error; --margin must be >= 0\n";
            return 2;
        }
    }

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

    std::cout << std::fixed << std::setprecision(3);

    // Calculations using the above inputs and predefined functions
    double d_react = reaction_distance(speed, reaction_time);
    double d_brake = compute_brake_distance(speed, decel);
    double d_total = d_react + d_brake;

    bool brake = needs_brake(distance_to_obs, speed, reaction_time, decel, margin);
    double d_total_with_margin = d_total * (1.0 + margin);
    std::cout << "Total distance (+margin):     " << d_total_with_margin << " m\n";

    std::cout << "\nPolicy margin used:           " << (margin * 100.0) << " %\n";
    std::cout << "\nReaction distance:            " << d_react  << " m\n";
    std::cout << "Brake distance:               " << d_brake   << " m\n";
    std::cout << "Total distance (no margin):   " << d_total   << " m\n";
    std::cout << "Total distance (+" << (margin * 100.0) << "% margin): " 
              << d_total_with_margin << " m\n";
    std::cout << "Decision (margin=" << (margin * 100.0) << "%): "
              << (brake ? "BRAKE" : "OK") << "\n";


    return 0;
}