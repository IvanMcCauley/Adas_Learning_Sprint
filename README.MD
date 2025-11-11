# ADAS Learning Sprint 🚗💡

### A 4-week deep-dive into the foundations of modern autonomous driving systems  
From physics and C++ to ROS 2 integration, latency analysis, and sensor fusion theory.

---

## 🧭 Overview
This repository documents my **ADAS Learning Sprint** - an intensive, self-directed program to bridge my **Mechatronic Engineering background** with the **software and systems knowledge** used in real-world ADAS and autonomy stacks.

What began as a simple braking-distance exercise in C++ grew into a full multi-stage pipeline:
1. **[braking_decision_lib](https://github.com/IvanMcCauley/braking_decision_lib)** - C++17 library + CLI/tests implementing stopping-distance logic.  
2. **[ros2_brake_decider](https://github.com/IvanMcCauley/ros2_brake_decider)** - ROS 2 C++ node with parameters, pub/sub, and real-time braking decisions at 20 Hz.  
3. **[ros2_brake_validation_sim](https://github.com/IvanMcCauley/ros2_brake_validation_sim)** - Python longitudinal simulation + replay, validating end-to-end system behavior.  

Once that pipeline was complete, the sprint shifted into **Stage 2** - developing systems-level awareness of ADAS architecture, QoS behavior, latency, timing, Kalman filters, and sensor fusion design patterns.

---

## 📘 What this repo contains
- **[log/README.md](log/README.md)** - full day-by-day log (setup, lessons, mistakes, and breakthroughs).  
- **Personal study notes** under `Sprint_02-Systems_Architecture_Notes/` - covering QoS tuning, timing, fusion, and autonomy stack design.  
- All code files and links to related repos.

---

## ⚙️ Learning focus by stage

### **Stage 1 - Foundations**
- Environment setup (Linux, CMake, VS Code, Git, WSL2).  
- C++ fundamentals: memory safety, const correctness, modularity, testing.  
- Vehicle physics + braking distance formulas.  
- Core math refresh: derivatives, rotation matrices, control intuition.

### **Stage 2 - Applied ROS 2 Development**
- Node lifecycle, publishers/subscribers, parameters, YAML config, launch files.  
- Implemented live braking decision node (20 Hz) and validated closed-loop via Python sim.  
- Added CSV logging, Matplotlib visualization, and MP4 replay generation.

### **Stage 3 - Systems & Architecture Thinking**
- Explored ROS 2 QoS profiles (reliability, durability, lifespan).  
- Measured period jitter and end-to-end latency in live nodes.  
- Modeled ADAS data flow from sensors → perception → planning → control.  
- Studied Kalman filtering, covariance tuning, and multi-sensor fusion concepts.  
- Developed intuition for timing budgets (~50 ms sensor→command) and freshness guarantees.

---

## 💡 Takeaways
- Deepened understanding of how **individual modules connect into a real-time ADAS pipeline**.  
- Built confidence designing and validating **deterministic, safety-critical logic** under timing constraints.  
- Strengthened hands-on experience in **C++17**, **ROS 2**, and **Python-based simulation**.  
- Cultivated a **systems-glue mindset** - linking perception, decision, and control with measurable timing and data-integrity guarantees.

---


