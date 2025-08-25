# braking_decision_lib

Tiny C++17 library + CLI for basic longitudinal safety checks (SI units).
Focus: reaction distance, brake distance, and a simple “brake now?” decision.

## Equations (SI)
- Reaction distance: $d_r = v \cdot t_r$ (m)
- Brake distance (const decel): $d_b = \frac{v^2}{2a}$ (m), $a > 0$
- Decision with margin $m$: brake if $(d_r + d_b)(1+m) \ge D_{\text{obs}}$


## Build
```
mkdir -p build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
make
```
## Run CLI
```
cd build
./brake_cli
./brake_cli --margin 0.20   # use a 20% safety margin, or can change to any value you prefer
```
## Run tests (assert-based)
```
cd build
make braking_tests
./braking_tests   # silent on success
```
## Layout
```
include/braking/braking.hpp   # declarations (units & preconditions)
src/braking.cpp               # definitions (assert guards)
app/brake_cli.cpp             # tiny interactive CLI
tests/test_braking.cpp        # assert-based checks
CMakeLists.txt                # library + CLI + tests
```
