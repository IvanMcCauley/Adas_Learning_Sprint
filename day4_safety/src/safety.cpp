// day4_safety/src/safety.cpp
#include <iostream>
#include <cmath>
using namespace std;

// d_brake = v^2 / (2a), with a > 0
double compute_brake_distance(double speed, double decel) {
    return (speed * speed) / (2.0 * decel);
}

// d_react = v * t, with t >= 0
double reaction_distance(double speed, double reaction_time) {
    return speed * reaction_time;
}

int main() {
    // All units: speed in m/s, decel in m/s^2, time in s, distances in m.
    double speed, reaction_time, decel, distance_to_obstacle;

    cout << "Speed (m/s): ";
    while (!(cin >> speed) || speed < 0.0) {
        cin.clear(); cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Enter a non-negative speed (m/s): ";
    }

    cout << "Reaction time (s): ";
    while (!(cin >> reaction_time) || reaction_time < 0.0) {
        cin.clear(); cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Enter a non-negative reaction time (s): ";
    }

    cout << "Braking deceleration (m/s^2): ";
    while (!(cin >> decel) || decel <= 0.0) {
        cin.clear(); cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Enter a positive deceleration (m/s^2): ";
    }

    cout << "Distance to obstacle (m): ";
    while (!(cin >> distance_to_obstacle) || distance_to_obstacle < 0.0) {
        cin.clear(); cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Enter a non-negative distance (m): ";
    }

    // Compute components
    double d_react = reaction_distance(speed, reaction_time);
    double d_brake = compute_brake_distance(speed, decel);
    double d_total = d_react + d_brake;

    // Report
    cout << "\n--- Results ---\n";
    cout << "Reaction distance: " << d_react  << " m\n";
    cout << "Braking distance : " << d_brake  << " m\n";
    cout << "Total stop dist  : " << d_total  << " m\n";
    cout << "Obstacle dist    : " << distance_to_obstacle << " m\n";

    bool must_brake = (distance_to_obstacle < d_total);
    cout << "\nDecision: " << (must_brake ? "BRAKE NOW" : "No immediate brake") << "\n";
}
