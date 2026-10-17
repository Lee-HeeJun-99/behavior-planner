# Right-turn obstacle regression fixtures

Recorded scenes from 10.17.4 at 30.0219 s (route index 241) and 10.17.6 at 63.7091 s (index 992). Exported from the current closed loop 0-0_loop.txt map with 0.75 m smoothing. UTM positions were translated by the nearest reference point; yaw and curvature are unchanged.

Each file contains: vehicle x/y/yaw and obstacle count; that many obstacle x/y pairs; then 300 reference x/y/yaw/curvature rows. Obstacles use the current 0.35 m model radius. 10.17.4 position uses the recorded ego marker; 10.17.6 heading and obstacle centers use recorded visualization markers. These fixtures test downstream planning, not raw perception or heading estimation.

All original constant-offset candidates fail these scenes. The new variable-offset fallback must produce a full-length path satisfying the same collision and curvature constraints. An additional blocking obstacle must invalidate the selected and retained paths.
