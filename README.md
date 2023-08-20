# A Fully Autonomous Heterogeneous Drone Swarm for Collaborative Target Pursuit
---
![leader](leader.gif)

Leader drone's view (detects wingman and stop sign)

![wingman](wingman.gif) 

Wingman drone's view (detects stop sign only)

## Hardware Overview
---
The leader drone hardware is the same as used in the single autonomous drone project (see master branch of this repository). It uses an NVIDIA Jetson Xavier NX GPU for an onboard computer and an Intel Realsense d435i camera.

The wingman drones are DJI Tello EDU drones spray-painted red to be more distinguishable from the background environment. These mini-drones carry an optical camera and send their camera stream to the leader drone via a wifi link. After processing the video, the leader drone sends movement commands back to the wingmen. 

## Software Organization
---

