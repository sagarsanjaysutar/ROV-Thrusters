# ROV Thruster Control

This is a ROS2 package that provides

1. Ability to control an ROV's movement underwater
2. Read frames from a USB Camera.

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

## Supported ROV Movements

1. Heave: ROV going vertically up & down under the water.
2. Sway: ROV moving it's entire body in a horizontally left & right movement.
3. Surge: ROV going foward or backward.
4. Roll: ROV's body rolling from east to west and visa versa.
5. Yaw: ROV moving left & right.

The following 4 thrusters control the horizontal movement i.e. Sway, Surge & Yaw.

1. T1 represents front left thurster.
1. T2 represents front right thurster.
1. T3 represents rear left thurster.
1. T4 represents rear right thurster.

The following 2 thrusters control the vertical movement i.e roll & hieve.

1. T5 represents left thurster.
1. T6 represents right thurster.

## Key mapping

- w / s → Surge (forward / backward)
- a / d → Sway (right / left)
- u / o → Yaw (left / right turn around vertical axis)
- q / e → Heave (up / down)
- j / l → Roll (left / right)

## Running the project

There are 2 executables, one associated with ROV Thruster control & another with reading web cam frames.

1. To control ROV thruster,

   1. In a new terminal, `cd` into compiled ArduPilot/ArduSub directory.
   2. `sim_vehicle.py --console --map` launch ArduSub
   3. In a new terminal launch Mavros node `ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@127.0.0.1:14555 -p target_system_id:=1 -p target_component_id:=1`
   4. In a new terminal, `cd` in project directory.
   5. `colcon build`
   6. `source install/setup.bash`
   7. `ros2 run rov_project thruster_control`.
   8. Control the ROV using given key maps.

1. To read web cam frames,

   1. In a new terminal, `cd` in project directory.
   2. `source install/setup.bash`
   3. `ros2 launch rov_project webcam.launch.py`.
