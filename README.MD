# ADAS Learning Sprint 🚗💻

**What this is:**  
A personal learning log where I tracked my sprint to further close the gap between my Mechatronic Engineering background and the skills needed for ADAS/autonomy roles.  

The sprint produced a 3-stage braking decision system:  
1. **[braking_decision_lib](https://github.com/IvanMcCauley/braking_decision_lib)** - C++17 stopping-distance library + CLI/tests  
2. **[ros2_brake_decider](https://github.com/IvanMcCauley/ros2_brake_decider)** - ROS 2 C++ node (params, pub/sub, 20 Hz decisions)  
3. **[ros2_brake_validation_sim](https://github.com/IvanMcCauley/ros2_brake_validation_sim)** - Python 1D sim + replay, closed-loop validation  

---

## Full log
The raw day-by-day notes have been moved to [log/README.md](log/README.md).  

---

## TL;DR - Why this repo exists
- The three repos above contain the actual code and full documentation.  
- **This repo is my day-by-day log**: setup, math refresh, C++ drills, ROS 2 basics, and eventually building + validating the full pipeline.  
- I’m keeping it public as a record of the process.

---

## Focus areas during the sprint
- C++ fundamentals (memory safety, modularity, headers/sources, testing, CMake)  
- Core math/physics for stopping distance and control  
- ROS 2 nodes, topics, parameters, timers, QoS  
- Python simulation and replay for validation  

