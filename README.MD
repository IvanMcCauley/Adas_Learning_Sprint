# Coding Sprint: Closing the Gap for ADAS & Autonomy Roles 🚗💻

This repository tracks my coding sprint to close the gap between my current skills and what’s needed for real-world ADAS and autonomous systems work.

I’m a Mechatronic Engineering graduate with project experience in YOLOv8 detection, BEV transformation, and A* path planning. Now I’m focusing on building deeper fluency in C++ and Python to meet the demands of embedded systems, planning, and perception roles in autonomy.

The repo is where I:
- ✅ Practice critical programming skills in C++ and Python  
- ✅ Rebuild core math/control foundations for robotics  
- ✅ Complete focused micro-projects tied to planning, perception, and control  
- ✅ Track progress day-by-day 
- ✅ Build tangible fluency through hands-on code  

🔍 **Focus areas**: C++ fundamentals, embedded logic, Python simulation, A*, PID, ROS 2, and real-time decision-making.  

📌 Updated frequently — each day’s work is logged and visible here.

# Projects 
[braking_decision_lib](https://github.com/IvanMcCauley/braking_decision_lib) - C++17 lib + CLI, CSV mode, tests, micro-benchmark.

## Day 1 - Environment Setup + Core Math Refresh

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


## Day 3 - C++
- Implemented: basics, I/O, conditions, loops (for/while/do-while), arrays, calculator
- Key skills: `cin` validation, `switch`, edge cases, `-Wall -Wextra`
- Code: `Week1/day3_cpp/*`

## Day 4 – C++ Memory Safety & Const
- Pointers & references basics (`pointers_refs.cpp`)  
- Pass-by-pointer vs pass-by-reference (`2_4a`, `2_4b`)  
- Return by reference (`2_5_return_by_ref.cpp`)  
- Dynamic allocation with `new`/`delete` (`dynamic.cpp`, `dynamic_array.cpp`)  
- `std::vector` basics + dataset return (`vector_basic.cpp`, `2_7_*.cpp`)  
- Memory pitfalls: dangling pointer, double delete (`2_6_*`)  
- Safe fixes: return by value, vectors  
- Const correctness: const params, const pointers, const member fns (`3_*.cpp`)  
- Capstone: ADAS-style braking/reaction distance + `needs_brake()` (`safety.cpp`) 

## Day 5 - C++ modularity (braking_decision_lib)
- Built `day5_braking/`: library + CLI + tiny tests from Day 4 `safety.cpp`.
- Practiced: headers vs sources, default 10% margin, `assert` guards, CMake lib/exe.
- Run: `cd day5_braking && mkdir -p build && cd build && cmake .. && make && ./brake_cli && ./braking_tests`
- Docs: see `day5_braking/README.md`.

## Day 6 - C++ modularity II (tests + margin)
- Added `--margin` flag to CLI; extra margin tests; optional strict build (`-Werror`).
- Run: `cd day5_braking/build && cmake -DBRK_STRICT=ON .. && make && ./brake_cli --margin 0.2 && ./braking_tests`

## Day 8 - CSV I/O
Added `--csv` mode to the braking CLI; learned file I/O (`ifstream`), line/token parsing (`getline`, `stringstream`), and safe number conversion (`stod`). Added tiny fixtures + a shell test that diffs expected vs actual output.

## Day 9 
Added input clamping/validation to the braking CLI (interactive + `--csv`), and a tiny benchmark (~14.6 ns/op over 1e6 evals on WSL2, g++ -O3 -DNDEBUG).

## Day 10 — Packaging & polish
Added MIT license, install rules (CMake), local install layout, and tagged v0.1.0. All CSV tests green.

---

## Day 11 - ROS 2 setup
Installed ROS 2 Humble (ros-base) on WSL2, created `~/ros2_ws` colcon workspace, and scaffolded first package `ros2_brake_decider`. Verified build and discovery (`ros2 pkg list`).
- **Reflection:**  Before today i was pretty much new to ROS, I learned that ROS 2 isn’t an “app,” but a framework where you build small programs called *nodes* that talk over *topics* (data streams) and use *parameters* for config. My `ros2_brake_decider` will be one such node, listening to speed + distance topics and publishing a brake command.
**System view:** In ROS 2, sensor nodes publish `/ego_speed` and `/obstacle_distance`, and my `ros2_brake_decider` node will subscribe, run the braking math, and publish a `/brake_cmd` decision.

## Day 12 – First real ROS 2 parameters
- Started by figuring out why nodes are usually written as classes. Realized it’s so they can actually hold state (like params, callbacks, timers) instead of cramming everything into `main()`.
- Declared my first parameters (`reaction_time`, `decel`, `safety_margin`) with defaults and learned how to read them at startup.
- Found out the hard way that parameters don’t update on their own → had to learn about `add_on_set_parameters_callback`.
- Added simple safety checks after testing: `reaction_time ≥ 0`, `decel > 0`, `0 ≤ safety_margin ≤ 1`.
- Ran into Humble-specific compiler errors (`SetParametersResult` not found) and only fixed it after digging into the right namespace (`rcl_interfaces::msg::SetParametersResult`).
- Also discovered the typical ROS workflow: run the node in one terminal, and use a second terminal to set params and watch the first terminal log updates or rejections. 

## Day 13 – First real ROS 2 topics (pub/sub)
- Subscribed to `/ego_speed` and `/obstacle_distance` (`std_msgs/Float64`) and used `ros2 topic pub` to feed fake values. First time I saw the callbacks actually print → proper “ah, they’re talking” moment.
- Added my first publisher: `/brake_cmd` (`std_msgs/Bool`) ticking at 20 Hz. Weirdly satisfying to see `false` spam the terminal nonstop, feels like the node has a pulse now.
- Ran into dumb mistakes (typo’d `std_msgs` as `std_msg`, forgot the underscore on a member, and even opened the wrong path once). Fixed them by checking includes, CMake deps, and running `ros2 node info` to see what was actually wired.
- Big takeaway today: **topics = live event streams**, **parameters = config knobs**. Also got more used to the ROS workflow: one terminal runs the node, another pokes it with `pub/echo/hz`.


