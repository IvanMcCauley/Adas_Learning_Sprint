# Day 2 – Physics & Control Basics

## Newtonian Mechanics
- F = m * a → net force makes the car accelerate.
- Example: 1000kg car, 2000N net force → a = 2 m/s².
- Heavier car = lower acceleration for same force.

## Kinematics
- v = u + a*t  
- s = u*t + 0.5*a*t²  
- v² = u² + 2*a*s  
- Good for predicting stopping distance.

## Rotational Motion
- Torque = I * α  
- Wheel speed: v = ω * r  
- Turning steering wheel changes torque → changes wheel angle → affects turning radius.

## Friction
- F_friction = μ * N  
- Static friction keeps tires gripping.  
- Higher μ or weight = more traction.  
- If force > friction limit → skid.

## Basic Vehicle Dynamics
- Bicycle model: car as 2 wheels (front steers).  
- Turning radius depends on steering angle & speed.  
- Too much centripetal force → loss of grip.

## Control Theory – PID
- P: reacts to current error.  
- I: fixes long-term small error.  
- D: slows down changes, reduces overshoot.
- Example: Cruise control - P adds throttle, I fixes small persistent gap, D prevents overshoot.

+-----------+
| Setpoint |
+-----------+
|
v
+-----------+ +---------+
| Controller|-----> | Plant |
| (PID) | | (Car) |
+-----------+ +---------+
^ |
| v
+-----------+ +---------+
| Feedback |<------| Output |
+-----------+ +---------+
