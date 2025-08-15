# Coding Sprint: Closing the Gap for ADAS & Autonomy Roles 🚗💻

This repository tracks my personal upskilling sprint focused on mastering Python and C++ for real-world autonomous systems.

I’m a Mechatronic Engineering graduate with hands-on project experience, including YOLOv8 detection, BEV transformation, and A* path planning, and I’m now deepening my low-level programming expertise to meet the demands of ADAS, robotics, and autonomous vehicle engineering roles.

To close this gap and become fully job-ready for high-impact engineering positions, I’m using this repo to:

- ✅ Practice critical programming skills in C++ and Python
- ✅ Rebuild core math/control foundations for robotics
- ✅ Complete focused micro-projects tied to planning, perception, and control
- ✅ Log daily/weekly progress with full transparency (`log.md`)
- ✅ Build tangible fluency through hands-on code

This sprint is part of a larger plan to become an **independent, high-leverage AV engineer** - fluent in the full autonomy stack and able to contribute meaningfully to mission-critical systems.

🔍 **Focus areas**: C++ fundamentals, embedded logic, Python simulation, A*, PID, ROS 2 (soon), and real-time decision-making.

📌 Updated frequently. Reflects learning milestones.

## Day 1 – Environment Setup + Core Math Refresh

- Set up Python, C++, Git, VS Code, Linux (WSL2)
- Wrote some basic python and C++ scripts to get warmed up
- Reviewed calculus (v = ds/dt), 2D rotation matrices
- Started PID controller summary in notes/control.md


## Day 2 - Physics Refresh & C++ Work
- Reviewed key physics concepts for vehicle dynamics and control theory and added Physics_Notes.md
- Verified C++ environment setup (Linux toolchain + VS Code integration).
- Compiled and ran first test program to confirm toolchain functionality.
- Configured VS Code debugger for step-by-step execution.
- Maintained organized folder structure (`Week1/day2_cpp`) for ongoing C++ exercises.


## Day 3 – C++ Fundamentals: Types, I/O, Control Flow, Loops, Arrays

Focused, hands-on C++ reps to solidify the 20% I’ll use 80% of the time in robotics/ADAS. Wrote and exercised small programs end-to-end (spec → code → compile → run → debug), keeping everything in `Week1/day3_cpp/`.

**Highlights**
- **Types & formatted output:** `int/double/char/bool`, `cout`, `boolalpha`, float vs double precision, unsigned wrap-around behavior.
- **Input parsing:** `cin` basics and a small recovery loop with `cin.fail() / clear() / ignore()` for robust console input.
- **Branching:** sign + parity checks with `if / else if / else`.
- **Loops:** `for / while / do-while`; built factorial `n!` and `sum(1..n)`; verified edge cases (`n=0`).
- **Arrays:** fixed-size array input (5 ints), aggregate with `sum` and `max`; worked on why `maxv` must initialize from data (`arr[0]`).
- **Mini calculator:** looped CLI calculator (`a op b`) using `switch` with divide-by-zero and unknown-operator guards.
- **Tooling & hygiene:** VS Code Remote-WSL workflow stabilized; compiled with `-std=c++17` (and warnings `-Wall -Wextra`); expanded `.gitignore` to exclude build artifacts.

**Artifacts (`Week1/day3_cpp/`)**
- `basics.cpp` – types, printing, `boolalpha`, precision demo  
- `io.cpp` – console input; simple arithmetic on user data  
- `cin_fail.cpp` – input validation & recovery loop  
- `conditions.cpp` – sign & even/odd classification  
- `loops.cpp` – for/while/do-while; factorial & series sum  
- `array.cpp` – read 5 values; `sum` + `max` (avg next)  
- `calculator.cpp` – looped text calculator with `switch` and error guards  
- `puzzles.cpp` – common C++ gotchas (assignment in `if`, `static` storage)

**What this proves**
- I can take a plain-English spec and deliver working C++ with clean control flow, basic data structures, and guardrails for bad input—compiled, tested, and versioned in a Linux toolchain.

**Next (Day 4 preview)**
- Functions, headers, and project structure; migrate arrays → `std::vector`, introduce `<algorithm>`, and start a small CMake project to prep for ROS 2.


