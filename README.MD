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

## Day 4 – C++ Memory Safety & References
- Practiced pointers (`*`, `&`) and how they let you modify variables through addresses  
- Compared pass-by-pointer vs pass-by-reference in functions  
- Tried dynamic allocation with `new` / `delete`  
- Looked at `std::vector` as a safer alternative to raw arrays  
- Wrote a small `element_at()` function returning by reference  
- Checked a few memory pitfalls:
  - dangling pointer (`2_6_dangling.cpp`) → undefined behavior  
  - double delete (`2_6_double_delete.cpp`) → undefined behavior  
  - return by value (`2_6a_return_value.cpp`) → safe fix  
  - dynamic vectors / return dataset (`2_7_*.cpp`) → safe, idiomatic  
- **Key takeaway**: better feel for what’s safe vs unsafe in C++ memory handling.  
- Code: `day4_safety/src/*`



