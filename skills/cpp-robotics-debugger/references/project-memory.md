# Project Memory Reference

Use a project memory file to improve continuity across sessions without bloating every prompt.
Prefer `.codex/project-memory.md` when writable. If repository policy keeps `.codex/` read-only, use `PROJECT_MEMORY.md` at the repository root or ask the user where to store it.

## What To Store

- Durable architecture facts: package layout, main executables, hardware devices, protocol notes, critical threads, launch files.
- Known environment constraints: ROS distro, Ubuntu version, compiler, required SDKs, hardware availability.
- Recurring debugging commands: known-good build commands, launch commands, topic inspection commands.
- Important decisions: why a fix or pattern was chosen, especially if it prevents repeating a failed approach.
- Current unresolved issues with date, symptom, suspected component, and next probe.

## What Not To Store

- Large logs, full stack traces, complete file contents, or temporary command output.
- Secrets, credentials, device serial numbers if sensitive, network keys, or private tokens.
- Speculation that is not labeled as a hypothesis.

## Suggested Format

```markdown
# Project Memory

## Architecture
- `package/name`: role, entry points, important dependencies.

## Hardware And Runtime
- Sensor/device facts, transport, expected topics/frames.

## Build And Test
- Known-good commands and caveats.

## Debugging Notes
- YYYY-MM-DD: Symptom -> evidence -> decision/fix -> residual risk.

## Open Questions
- Question, why it matters, next command/file to inspect.
```

## Update Rules

- Keep entries short enough to scan.
- Merge duplicate facts instead of appending conflicting notes.
- Add dates for issue-specific notes.
- Remove stale hypotheses once disproven.
