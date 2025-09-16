# ADAS Learning Sprint - Day-by-Day Log

This folder contains my **raw daily notes** from the sprint.  
They’re here for transparency and reflection - if you just want the runnable code, see the Stage 1–3 repos in the [main README](../README.MD).

---

## Day 1 - Environment Setup + Core Math Refresh
- Set up Python, C++, Git, VS Code, Linux (WSL2).
- Wrote basic Python and C++ scripts to get warmed up.
- Reviewed calculus (v = ds/dt), 2D rotation matrices.
- Started PID controller notes in `notes/control.md`.

---

## Day 2 - Physics Refresh & C++ Work
- Reviewed vehicle dynamics and control theory → `Physics_Notes.md`.
- Verified C++ toolchain (Linux + VS Code).
- Compiled and ran first test program.
- Configured debugger for step-by-step execution.
- Started organized folder structure (`Week1/day2_cpp`).

---

## Day 3 - C++ Basics
- Implemented: basics, I/O, conditions, loops, arrays, calculator.
- Key skills: `cin` validation, `switch`, edge cases, `-Wall -Wextra`.
- Code in `Week1/day3_cpp/*`.

---

## Day 4 - C++ Memory Safety & Const
- Pointers & references basics (`pointers_refs.cpp`).
- Pass-by-pointer vs pass-by-reference (`2_4a`, `2_4b`).
- Return by reference (`2_5_return_by_ref.cpp`).
- Dynamic allocation with `new`/`delete` (`dynamic.cpp`, `dynamic_array.cpp`).
- `std::vector` basics (`vector_basic.cpp`, `2_7_*.cpp`).
- Memory pitfalls: dangling pointer, double delete (`2_6_*`).
- Safe fixes: return by value, vectors.
- Const correctness: const params, const pointers, const member fns (`3_*.cpp`).
- Capstone: braking/reaction distance + `needs_brake()` (`safety.cpp`).

---

## Day 5 - C++ Modularity (braking_decision_lib)
- Built `day5_braking/`: library + CLI + tiny tests from Day 4 `safety.cpp`.
- Practiced: headers vs sources, 10% margin default, `assert` guards, CMake lib/exe.
- Run:  
  ```bash
  cd day5_braking && mkdir -p build && cd build
  cmake .. && make
  ./brake_cli && ./braking_tests

---

## Day 6 - C++ Modularity II
- Added `--margin` flag to CLI.
- Added extra margin tests.
- Optional strict build (`-Werror`).
- Run:
```bash
cd day5_braking/build
cmake -DBRK_STRICT=ON ..
make
./brake_cli --margin 0.2
./braking_tests
```

---

## Day 8 - CSV I/O
- Added `--csv` mode to braking CLI.
- Learned: `ifstream`, `getline`, `stringstream`, `stod`.
- Added fixtures + shell test diffing expected vs actual output.

---

## Day 9 - Input Validation & Benchmark
- Added input clamping/validation (interactive + `--csv`).
- Added micro-benchmark (~14.6 ns/op over 1e6 evals, g++ -O3, WSL2).

---

## Day 10 - Packaging & Polish
- Added MIT license, install rules (CMake).
- Local install layout.
- All CSV tests green.

---

## Day 11 — ROS 2 Setup
- Installed ROS 2 Humble (ros-base) on WSL2.
- Created `~/ros2_ws`, scaffolded first package (`ros2_brake_decider`).
- Verified build/discovery with `ros2 pkg list`.
- Reflection: Learned ROS 2 is a middleware framework (nodes, topics, parameters), not a single app.

---

## Day 12 - ROS 2 Parameters
- Learned why nodes are usually classes (state: params, callbacks, timers).
- Declared parameters: `reaction_time`, `decel`, `safety_margin`.
- Added validation: `reaction_time ≥ 0`, `decel > 0`, `0 ≤ safety_margin ≤ 1`.
- Debugged Humble-specific quirk (`SetParametersResult` namespace).
- Practiced workflow: run node in one terminal; use another to param set and watch logs.

---

## Day 13 - ROS 2 Pub/Sub
- Subscribed to `/ego_speed` and `/obstacle_distance` (`std_msgs/Float64`).
- Used `ros2 topic pub` to feed fake values -> first working callbacks.
- Added publisher `/brake_cmd` (`std_msgs/Bool`) at 20 Hz.
- Debugged typos + CMake deps with `ros2 node info`.
- Takeaway: Topics = event streams; Params = config knobs.

---

## Day 14 - First Brake Decisions
- Stored last values in class so timer could always run.
- Added stopping-distance formula from Stage 1.
- Verified correct toggling of `/brake_cmd`.
- Tested live param changes and saw decision boundary move.

---

## Day 15 - Launch + Config
- Added `config/brake_params.yaml` and launch file.
- Learned to install non-code assets via CMake.
- Sanity-check: `ros2 param get` after launch -> YAML values loaded.

---

## Day 16 - Minimal ROS 2 Sim
- Built `longitudinal_1d_sim` (Python).
- `/brake_cmd` sub, 20 Hz timer, (x,v) update, CSV `t,x,v,d,brake_cmd,u`.
- Debugged executor crash (spun class not instance).
- First stop test: `/brake_cmd=True` -> stop ≈2.2 s, margin ≈57 m.

## Day 17 - Closed Loop + Plot
- Sim now publishes `/ego_speed` + `/obstacle_distance`.
- C++ node subscribes + publishes `/brake_cmd`.
- Added `make_plot.py` to plot v(t), d(t) + brake timeline.
- Observed chatter near threshold.
- Example result: SUCCESS at 8.5 s, margin ≈0.67 m.

## Day 18 - CSV → MP4 Visualizer
- Rendered sim CSV:
  - Left: car vs wall + brake light.
  - Right: v(t)/d(t) + moving cursor.
- Learned Matplotlib basics: GridSpec, axvline, ffmpeg writer.
- Installed ffmpeg to export MP4.

## Day 19 - Video Polish
- Added title, decision badge, Topics+Params block.
- Brake light square with z-order tweaks.
- HUD above world view.
- Plots with time cursor.
- Result: clear MP4 replay of closed-loop run.

---

# Sprint 2 - Systems-minded AV/ADAS (14 DAYS)
## Day 20 - ROS 2 graph, QoS, domains, bagging (operator skills)
- Got talker/listener running, read QoS per endpoint (RELIABLE/VOLATILE), tried BEST_EFFORT (worked) vs TRANSIENT_LOCAL (failed as expected), and isolated graphs with ROS_DOMAIN_ID (0 ↔ 17).  
- Recorded + replayed `/chatter` and proved the publisher was `/rosbag2_play…` - finally “get” QoS compatability and bag replay (also learned the bash `|`/`grep`/`||` basics).
- Wrote personal study notes in [Sprint_02-Systems_Architecture_Notes](https://github.com/IvanMcCauley/Adas_Learning_Sprint/tree/main/Sprint_02-Systems_Architecture_Notes) folder.

## Day 21 - ROS 2 Python node + QoS/params/launch
- Created ament_python pkg `planning101`; wrote `qos_probe` subscriber to `/chatter` with QoS from params (CLI wins over YAML).
- Fixed `setup.py` (`console_scripts` + `data_files`) so launch/params install under `share/…`; built with `--symlink-install` and ran via `ros2 run` + `ros2 launch`.
- Verified endpoint QoS with `ros2 topic info -v`.
- Learned `super().\__init__`, `self`, callbacks, `create_subscription(type, topic, callback, qos)` and why QoS mismatch = no messages.
- Wrote personal study notes in [Sprint_02-Systems_Architecture_Notes](https://github.com/IvanMcCauley/Adas_Learning_Sprint/tree/main/Sprint_02-Systems_Architecture_Notes) folder.

## Day 22 - Timing: latency vs period jitter
- Added `pinger` that stamps `Header` on `/ping`; `qos_probe` computes E2E latency (`now` - `msg.stamp`) and sliding-window mean/max.
- Observed ~0.7-1.6 ms typical latency with small spikes.
- Compared with `ros2 topic hz` (period jitter): `/ping` ~20 Hz with tiny jitter; `/chatter` ~1 Hz with ≈±25 ms.
- Proved changing a QoS param at runtime doesn’t rewire an existing sub (QoS fixed at creation).
- Now solid on timers, age-of-info vs jitter, and how to read QoS per endpoint.
- Wrote personal study notes in [Sprint_02-Systems_Architecture_Notes](https://github.com/IvanMcCauley/Adas_Learning_Sprint/tree/main/Sprint_02-Systems_Architecture_Notes) folder.
