# ROV Thruster Control

This is a ROS2 package that provides ability to control an ROV's movement underwater. This package covers 5 movements

1. Heave: ROV going vertically up & down under the water.
2. Sway: ROV moving it's entire body in a horizontally left & right movement.
3. Surge: ROV going foward or backward.
4. Roll: ROV's body rolling from east to west and visa versa.
5. Yaw: ROV moving left & right.

## ROV Layout

    Front (nose of ROV)

    T1             T2
      \            /
       \   ROV    /
       T5        T6
       /          \
      /            \
    T3             T4

    Rear (tail of ROV)

The following 4 thrusters control the horizontal movement i.e. Sway, Surge & Yaw.

1. T1 represents front left thurster.
1. T2 represents front right thurster.
1. T3 represents rear left thurster.
1. T4 represents rear right thurster.

The following 2 thrusters control the vertical movement i.e roll & hieve.

1. T5 represents left thurster.
1. T6 represents right thurster.

## ROV Movement

- w / s → Surge (forward / backward)
- a / d → Sway (right / left)
- u / o → Yaw (left / right turn around vertical axis)
- q / e → Heave (up / down)
- j / l → Roll (left / right)

## Running the project

As this is a ROS2 package, it is assumed that ROS2 is installed & sourced.

1. In a new terminal,
   1. `cd` in project directory.
   2. `colcon build`
   3. `source install/setup.bash`
   4. `ros2 run rov_project velocity_publisher`.
1. In a new terminal,
   1. `cd` in project directory.
   2. `source install/setup.bash`
   3. `ros2 run rov_project thruster_publisher`.
