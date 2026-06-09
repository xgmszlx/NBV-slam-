# Gazebo Models

Place the differential-drive RGB-D robot model and walking-person models here.

The first implementation milestone uses the Gazebo world placeholders to validate
launch wiring. The full reproduction should add:

- a differential-drive base with `/cmd_vel`;
- a `640 x 480` RGB-D camera matching the paper intrinsics;
- two walking-person actors with deterministic back-and-forth trajectories per environment.

