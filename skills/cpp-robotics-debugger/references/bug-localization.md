# Bug Localization Reference

## Token-Efficient Investigation

- Start with topology, not source dumps: packages, executables, library boundaries, node entry points, message types, topics, launch parameters, and hardware transport.
- Use `rg` first for symbols and strings. Search both declarations and use sites.
- Read headers before implementation when class ownership or public contracts matter.
- Read implementation around constructors, destructors, initialization, callbacks, parse loops, and thread entry points.
- Keep a short scratch model: symptom -> component -> call path -> suspect invariant -> evidence.

## ROS/Catkin Checklist

- Check `package.xml` and `CMakeLists.txt` for missing dependencies, include directories, link libraries, C++ standard, generated messages, and executable targets.
- Trace node startup from `main()` to object construction, parameter loading, publisher/subscriber setup, and spin loop.
- For launch/runtime issues, compare parameter names, namespaces, frame IDs, topic names, queue sizes, and remaps.
- For RViz/data visibility issues, verify frame IDs, timestamps, TF availability, topic type, message rate, and fixed frame.
- For catkin build errors, distinguish include-path errors, link errors, ABI/C++ standard mismatch, and generated-message ordering.

## C++ Robotics Failure Modes

- Lifetime: dangling references in callbacks, raw pointer ownership, destructor order, thread join/stop sequencing, use-after-close of device handles.
- Concurrency: data races between callback/spin thread and device read thread, non-atomic stop flags, callbacks holding locks during blocking I/O.
- Timing: invalid timestamps, stale sensor data, timeout units mismatch, busy loops without sleep, serial reads assuming full packets.
- Protocol parsing: off-by-one frame length, endian mismatch, checksum computed over wrong range, resync failures after partial/corrupt packets.
- Numeric robustness: degrees/radians mismatch, integer truncation, float comparison thresholds, invalid ranges, NaN propagation.
- ROS integration: wrong frame convention, missing covariance, inconsistent `header.stamp`, publishing before calibration/initialization.
- Build/config: include path shadowing, linking wrong library, missing `target_link_libraries`, stale generated files, untracked launch defaults.

## Evidence Standards

- Treat compiler output, logs, stack traces, and exact code references as facts.
- Treat domain expectations and inferred runtime behavior as hypotheses until validated.
- Prefer fixes that preserve public interfaces unless the bug is in the interface itself.
- Before editing shared SDK or driver code, inspect all call sites for implicit contracts.

## Validation Choices

- Run the narrowest build target available before full workspace builds.
- If hardware is unavailable, validate parser logic with existing samples, tests, or small deterministic fixtures.
- For thread/lifetime fixes, inspect shutdown paths and run available sanitizers only when the project already supports them or the user asks.
- Report what was not validated, especially physical sensor behavior.
