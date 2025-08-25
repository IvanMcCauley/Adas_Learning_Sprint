#pragma once 
/** 
 * Braking decision library - declarations only
 *
 * SI units:
 *  - speed: m/s
 *  - decel: m/s^2
 *  - time: s
 *  - distance m
 *
 * Functions are pure (no side effects) and dont allocate memory.
 * Definitions live in src/braking.cpp
 */

// Distance needed to brake from 'speed' to 0 under constant 'decel'
// Uses s=v^2/(2a). Precondition: decel > 0. 
double compute_brake_distance(double speed, double decel);

// Distance traveled during reaction delay: d=v*t
// Precondition: reaction time >=0
double reaction_distance(double speed, double reaction_time);

// Decide if we must brake now.
// Returns true when (reaction + brake)*(1+ safety_margin) >= distance_to _obstacle
// Typical safety_margin = 0.10 (10%). Preconditions: decel > 0, reaction_time >= safety_margin >=0
bool needs_brake(double distance_to_obstacle, 
                 double speed, 
                 double reaction_time, 
                 double decel, 
                 double safety_margin = 0.10);


