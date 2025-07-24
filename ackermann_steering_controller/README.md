## Ackermann Steering Controller ##

Controller for a ackermann steering drive mobile base. 

Detailed user documentation can be found in the controller's [ROS wiki page](http://wiki.ros.org/ackermann_steering_controller)

### New Feature: Steering Velocity Limits ###

This controller now supports limiting the maximum velocity of the steering actuator. This feature prevents sudden steering movements that could be harmful to the steering mechanism or cause instability.

#### Configuration Parameters ####

The steering velocity limits can be configured using the following parameters:

```yaml
steering:
  has_velocity_limits: true
  max_velocity: 2.0          # rad/s - maximum steering angle change rate
  min_velocity: -2.0         # rad/s - minimum steering angle change rate
  has_acceleration_limits: true
  max_acceleration: 4.0      # rad/s² - maximum steering acceleration
  min_acceleration: -4.0     # rad/s² - minimum steering acceleration
  has_jerk_limits: true
  max_jerk: 10.0            # rad/s³ - maximum steering jerk
  min_jerk: -10.0           # rad/s³ - minimum steering jerk
```

#### Usage ####

1. Set `has_velocity_limits: true` to enable steering velocity limiting
2. Configure `max_velocity` and `min_velocity` to set the maximum rate at which the steering angle can change
3. Optionally enable acceleration and jerk limits for smoother steering motion
4. See `config/example_steering_limits.yaml` for a complete configuration example

#### Benefits ####

- Protects steering hardware from sudden angle changes
- Provides smoother, more controlled steering motion
- Reduces mechanical stress on steering components
- Improves overall system stability
