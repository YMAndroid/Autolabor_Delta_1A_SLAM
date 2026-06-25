---
name: cpp-robotics-debugger
description: Token-efficient C++ robotics and ROS/catkin project analysis for bug localization, build failures, runtime crashes, sensor/driver issues, SLAM/navigation behavior, and code review. Use when working in C++ robot repositories, especially ROS nodes, SDKs, lidar/camera/IMU drivers, serial/network device code, CMake/catkin packages, threading, timing, memory lifetime, Eigen/PCL/OpenCV, or performance-sensitive debugging.
---

# C++ Robotics Debugger

Use this skill to inspect robotics C++ projects with a small, evidence-first context footprint.
Prefer repository indexing, targeted reads, and explicit hypotheses over broad file dumping.

## Default Workflow

1. Build a compact project map before reading large files:
   - Run `pwd`, `rg --files`, `find . -maxdepth 3 -type f`, or package-specific discovery.
   - Locate `CMakeLists.txt`, `package.xml`, launch files, node entry points, SDK boundaries, and hardware interfaces.
   - Summarize only the relevant modules and ownership boundaries.

2. Read narrow slices:
   - Use `rg` for symbols, error strings, topics, parameters, class names, and device protocol constants.
   - Open only the call path, adjacent headers, and tests/logs needed to validate the current hypothesis.
   - Avoid pasting entire files into context unless the file is short or its structure is unknown.

3. Localize bugs by evidence:
   - State the observed symptom, the suspected layer, and the evidence.
   - Track assumptions separately from facts found in code/logs.
   - Prefer one or two high-confidence fixes over speculative rewrites.

4. Validate with the cheapest reliable check:
   - Use compile checks, focused unit tests, `catkin_make`/`catkin build`, or static inspection depending on risk.
   - For hardware-dependent bugs, verify protocol parsing, timeout behavior, thread lifetime, and error handling without pretending hardware was tested.

5. Update memory when useful:
   - If the task reveals durable project facts, write or update a concise project memory file.
   - Keep memory factual, dated when helpful, and easy to grep.

## Reference Routing

- Read `references/bug-localization.md` for C++/ROS bug triage, common robotics failure modes, and investigation patterns.
- Read `references/project-memory.md` before creating or updating long-lived project notes.

## Output Style

When explaining findings, lead with the likely cause and file/line evidence.
Keep summaries compact: project map, evidence, fix, validation, residual risk.
When uncertain, name the next command or file that would reduce uncertainty most.
