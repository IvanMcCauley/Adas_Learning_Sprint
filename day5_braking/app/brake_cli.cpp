#include <iostream>
#include <iomanip>
#include <string>  //for std::string
#include <cstdlib> //for std::atof
#include <fstream>   // file input
#include <sstream>   // split a line by commas
#include "braking/braking.hpp"

// Process a CSV file of rows: speed,reaction_time,decel,obstacle_distance
// Prints: speed,reaction_time,decel,obstacle_distance,decision
static int process_csv(const std::string& path, double margin) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "Error: could not open CSV file: " << path << "\n";
        return 2;
    }

    std::string line;
    bool header_skipped = false;

    // Output header (useful if redirecting to a file)
    std::cout << "speed,reaction_time,decel,obstacle_distance,decision\n";

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') continue;

        if (!header_skipped) {   // skip first non-comment line (header row)
            header_skipped = true;
            continue;
        }

        std::stringstream ss(line);
        std::string a,b,c,d;
        if (!std::getline(ss, a, ',') ||
            !std::getline(ss, b, ',') ||
            !std::getline(ss, c, ',') ||
            !std::getline(ss, d, ',')) {
            std::cerr << "Warning: bad row (need 4 fields): " << line << "\n";
            continue;
        }

        try {
            double speed         = std::stod(a);
            double reaction_time = std::stod(b);
            double decel         = std::stod(c);
            double distance      = std::stod(d);

            bool brake = needs_brake(distance, speed, reaction_time, decel, margin);

            // CSV output (no spaces)
            std::cout << speed << ","
                      << reaction_time << ","
                      << decel << ","
                      << distance << ","
                      << (brake ? "BRAKE" : "OK") << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Warning: parse error on row: " << line
                      << " (" << e.what() << ")\n";
            continue;
        }
    }
    return 0;
}


// Simple interactive CLI for braking calculations (SI units)
int main(int argc, char* argv[]) {
    
   
    // If user runs: ./brake_cli --margin 0.2 --> margin becomes 0.2 (20%)
    // ---- new arg parsing ----
    double margin = 0.10;       // default 10%
    std::string csv_path;       // empty = no CSV mode

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--margin") {
            if (i + 1 >= argc) { std::cerr << "Error: --margin needs a value\n"; return 2; }
            margin = std::atof(argv[++i]);      // use next token (e.g. "0.2")
            if (margin < 0.0)   { std::cerr << "Error: --margin must be >= 0\n"; return 2; }
        } else if (arg == "--csv") {
            if (i + 1 >= argc) { std::cerr << "Error: --csv needs a path\n"; return 2; }
            csv_path = argv[++i];               // path to CSV file
        } else {
            std::cerr << "Unknown arg: " << arg << "\n";
            std::cerr << "Usage: ./brake_cli [--margin <fraction>] [--csv <file>]\n";
            return 2;
        }
    }

    // If CSV mode selected, process file and exit early.
    if (!csv_path.empty()) {
        return process_csv(csv_path, margin);
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