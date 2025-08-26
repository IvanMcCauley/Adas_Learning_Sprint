# braking_decision_lib

Tiny C++17 library + CLI for basic longitudinal safety checks (SI units).
Focus: reaction distance, brake distance, and a simple “brake now?” decision.

## Equations (SI)
- Reaction distance: $d_r = v \cdot t_r$ (m)
- Brake distance (const decel): $d_b = \frac{v^2}{2a}$ (m), $a > 0$
- Decision with margin $m$: brake if $(d_r + d_b)(1+m) \ge D_{\text{obs}}$
**SI units:** speed m/s, time s, decel m/s² (positive magnitude), distance m, margin = fraction (e.g., 0.10 = 10%).

---

## Layout
```
day5_braking/
├── include/
│   └── braking/
│       └── braking.hpp          # declarations (free fns + class)
├── src/
│   └── braking.cpp              # definitions + input asserts
├── app/
│   └── brake_cli.cpp            # interactive CLI (supports --margin)
├── tests/
│   └── test_braking.cpp         # tiny assert-based tests
├── CMakeLists.txt
└── README.md

```
## Build
From this folder:
```
mkdir -p build && cd build
cmake -DBRK_STRICT=ON ..   # optional strict mode: warnings -> errors
make -j
```

## Run CLI
Interactive run (default margin = 0.10):
```
./brake_cli
```

Custom margin (e.g., 20%):
```
./brake_cli --margin 0.20
```

## Example Session
```
Enter speed (m/s): 20
Enter reaction time (s): 3
Enter decel magnitude (m/s^2): 10
Enter distance to obstacle (m): 90

Policy margin used:           20.000 %
Reaction distance:            60.000 m
Brake distance:               20.000 m
Total distance (no margin):   80.000 m
Total distance (+20.000% margin): 96.000 m
Decision (margin=20.000%):    BRAKE
```
## Tests
Build & run (silent on success):
```
cd build
make braking_tests -j
./braking_tests
```

## API (library)
Free functions (simple & stateless):
``` 
// SI units: speed (m/s), decel (m/s^2), time (s), distance (m)
double compute_brake_distance(double speed, double decel);
double reaction_distance(double speed, double reaction_time);
bool needs_brake(double distance_to_obstacle,
                 double speed,
                 double reaction_time,
                 double decel,
                 double safety_margin = 0.10);
```
Class value-type (store policy once, then query):
```
class BrakingDecision {
public:
    BrakingDecision(double reaction_time, double decel, double safety_margin=0.10);
    double reaction_time() const noexcept;
    double decel()         const noexcept;
    double safety_margin() const noexcept;

    double reaction_distance(double speed) const;
    double compute_brake_distance(double speed) const;
    double total_distance(double speed) const;
    bool needs_brake(double distance_to_obstacle, double speed) const;
};
```

Class usage
```
#include "braking/braking.hpp"
BrakingDecision bd(1.0, 5.0, 0.10);      // t=1s, a=5 m/s^2, m=10%
bool brake = bd.needs_brake(70.0, 20.0); // D=70 m, v=20 m/s
```

## Input contracts & guards
- reaction_time >= 0
- decel > 0 (positive magnitude)
- safety_margin >= 0
- speed >= 0, distance_to_obstacle >= 0
- Contracts are enforced with assert(...) in Debug builds (they terminate on violation).

## Notes
- Headers declare, sources define. You include the header; CMake builds the library and links definitions.
- CLI prints with fixed decimals; --margin is optional and validated (≥0).
- Strict build toggle: -DBRK_STRICT=ON adds -Werror to keep the code tidy.
