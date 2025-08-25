#include <cassert>
#include "braking/braking.hpp"

// Brake distance: s = v^2 / (2a). Precondition: decel > 0.
double compute_brake_distance(double speed, double decel) {
    assert(decel > 0 && "decel must be > 0");
    return (speed * speed) / (2.0 * decel);
}

// Reaction distance: d = v * t. Precondition: reaction_time >= 0.
double reaction_distance(double speed, double reaction_time) {
    assert(reaction_time >= 0 && "reaction_time must be >= 0");
    return speed * reaction_time;
}

// Decide if we must brake now.
// Returns true when (reaction + brake) * (1 + safety_margin) >= distance_to_obstacle.
bool needs_brake(double distance_to_obstacle,
                 double speed,
                 double reaction_time,
                 double decel,
                 double safety_margin) {
    assert(decel > 0 && "decel must be > 0");
    assert(reaction_time >= 0 && "reaction_time must be >= 0");
    assert(safety_margin >= 0 && "safety_margin must be >= 0");

    double total_stop = reaction_distance(speed, reaction_time)
                      + compute_brake_distance(speed, decel);

    total_stop *= (1.0 + safety_margin);
    return total_stop >= distance_to_obstacle;
}
