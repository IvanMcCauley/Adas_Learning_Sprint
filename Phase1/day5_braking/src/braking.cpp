#include <cassert>
#include "braking/braking.hpp"

// ---- Free functions ----

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

// ===== BrakingDecision class definitions =====

BrakingDecision::BrakingDecision(double reaction_time,
                                 double decel,
                                 double safety_margin)
    : reaction_time_(reaction_time),   // <-- initializer list (explained below)
      decel_(decel),
      safety_margin_(safety_margin)
{
    // input contracts on construction
    assert(reaction_time_ >= 0.0 && "reaction_time must be >= 0");
    assert(decel_ > 0.0 && "decel must be > 0 (magnitude)");
    assert(safety_margin_ >= 0.0 && "safety_margin must be >= 0");
}

double BrakingDecision::reaction_distance(double speed) const {
    assert(speed >= 0.0 && "speed must be >= 0");
    return speed * reaction_time_;
}

double BrakingDecision::compute_brake_distance(double speed) const {
    assert(speed >= 0.0 && "speed must be >= 0");
    return (speed * speed) / (2.0 * decel_);
}

double BrakingDecision::total_distance(double speed) const {
    return reaction_distance(speed) + compute_brake_distance(speed);
}

bool BrakingDecision::needs_brake(double distance_to_obstacle,
                                  double speed) const {
    assert(distance_to_obstacle >= 0.0 && "distance_to_obstacle must be >= 0");
    double total = total_distance(speed) * (1.0 + safety_margin_);
    return total >= distance_to_obstacle;
}
