## Steered Diff Drive Controller ##

Controller for a differential drive mobile base with constrained steering wheel.

Detailed user documentation can be found in the controller's [ROS wiki page](http://wiki.ros.org/steered_diff_drive_controller).

# Note on commanding with AckermannDrive Interface
It's based on Energy shaping method.
ackermann_vel.speed is linear velocity ONLY when the steering angle is zero.

When steering angle is present, ackermann_vel.speed is divided for linear velocity and angular velocity
aka. Ellipsoid Model

We can calculate max_angular_velocity from rotational_multiplier * linear_velocity 