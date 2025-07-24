## Steered Diff Drive Controller ##

Controller for a differential drive mobile base with constrained steering wheel.

Detailed user documentation can be found in the controller's [ROS wiki page](http://wiki.ros.org/steered_diff_drive_controller).

# Velocity Limits

The controller supports velocity limits for:
- Linear velocity (linear/x/*)
- Angular velocity (angular/z/*)  
- Steering velocity (steering/*)

Steering velocity limits can be configured using the following parameters:
```yaml
steering:
  has_velocity_limits: true
  max_velocity: 1.57  # rad/s (90 degrees/second)
  has_acceleration_limits: true
  max_acceleration: 3.14  # rad/s^2
  has_jerk_limits: false
  max_jerk: 0.0  # rad/s^3
```

# Note on commanding with AckermannDrive Interface
It's based on Energy shaping method.
ackermann_vel.speed is linear velocity ONLY when the steering angle is zero.

When steering angle is present, ackermann_vel.speed is divided for linear velocity and angular velocity
aka. Ellipsoid Model

We can calculate max_angular_velocity from rotational_multiplier * linear_velocity 