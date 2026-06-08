# External Runtime Files

This directory stores versioned copies of runtime files that currently live outside this ROS2 package.

Active runtime paths remain unchanged for now:

- `/home/addala/ros2_moveit_ws/LLM IA ASTER/aster_server.py`
- `/home/addala/ros2_moveit_ws/arduino/Play.ino`

Versioned copies:

- `external/aster_server/aster_server.py`
- `external/arduino/Play/Play.ino`

The copied `aster_server.py` must not contain a hard-coded Gemini API key. Set the key at runtime with:

```bash
export GEMINI_API_KEY=...
```

These copies are intended as a safe migration base. Do not assume the active launch path has moved until the runtime scripts are explicitly updated.
