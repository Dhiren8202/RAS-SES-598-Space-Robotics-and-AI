# First-Order Boustrophedon Navigator (Lawnmower Pattern) Using ROS2

## Turtlesim Output Image
![Screenshot from 2025-01-27 23-48-18](https://github.com/user-attachments/assets/0384207b-e959-4687-bf61-0bb3eca7019a)



---

## Overview

This project implements a first-order boustrophedon navigator for the Turtlesim in ROS2. The primary goal was to execute a precise lawnmower survey pattern while minimizing cross-track error and maintaining smooth motion.

Additionally, extra credit was attempted by defining a custom ROS2 message type to publish performance metrics.

---

## Performance Metrics

- **Final Parameters:**

  - `Kp_linear`: 2.8
  - `Kd_linear`: 0.02
  - `Kp_angular`: 11.8
  - `Kd_angular`: 0.03
  - `spacing`: 1.5

- **Performance Results:**

  - **Average Cross-Track Error**: 0.028 units
  - **Maximum Cross-Track Error**: 0.08 units
  - Smooth motion and clean cornering were achieved.

---

## Extra Credit Progress

### Custom Message Type Integration

As part of the extra credit, I implemented the initial steps of integrating a custom ROS2 message type to publish detailed performance metrics. Below is a summary of the work completed:

1. **Creating the Custom Message Package:**

   - A new package `custom_msgs` was created using the following commands:
     ```bash
     cd ~/Dhiren_ws/src
     ros2 pkg create custom_msgs --build-type ament_cmake --dependencies std_msgs
     ```
   - A message definition file `NavigatorMetrics.msg` was added with the following fields:
     ```
     float64 cross_track_error
     float64 current_velocity
     float64 distance_to_next_waypoint
     float64 completion_percentage
     string status_message
     ```
   - The `CMakeLists.txt` and `package.xml` files were updated to include the necessary configurations for generating custom messages.
   - The package was built successfully:
     ```bash
     cd ~/Dhiren_ws
     colcon build --packages-select custom_msgs
     ```

2. **Using the Custom Message in the Node:**

   - The `first_order_boustrophedon_navigator` package was modified to use the `custom_msgs` package. This included:
     - Adding `custom_msgs` as a dependency in `package.xml`.
     - Importing the custom message type in the Python node:
       ```python
       from custom_msgs.msg import NavigatorMetrics
       ```

I tried to proceed beyond this but was not successfully able to complete it yet. I have an idea of what to do and will work on it further. Once resolved, I plan to push the updated code.

---

## Parameter Adjustments

### Initial Parameters

- `Kp_linear`: 10.0
- `Kd_linear`: 0.1
- `Kp_angular`: 5.0
- `Kd_angular`: 0.2
- `spacing`: 1.0

### Adjustments Made

1. **Proportional Gain for Linear Velocity:**

   - Initial value: `10.0`
   - Final value: `2.8`
   - **Reason**: The high initial value caused oscillations and overshooting in the linear velocity. Reducing it improved stability and allowed smoother transitions between segments.

2. **Derivative Gain for Linear Velocity:**

   - Initial value: `0.1`
   - Final value: `0.02`
   - **Reason**: A lower derivative gain helped reduce abrupt changes in velocity, leading to smoother motion.

3. **Proportional Gain for Angular Velocity:**

   - Initial value: `5.0`
   - Final value: `11.8`
   - **Reason**: Increasing the angular proportional gain enabled sharper and more accurate turns, which minimized deviations during cornering.

4. **Derivative Gain for Angular Velocity:**

   - Initial value: `0.2`
   - Final value: `0.03`
   - **Reason**: The reduced derivative gain allowed for smoother adjustments in angular velocity without overcompensating.

5. **Spacing:**

   - Initial value: `1.0`
   - Final value: `1.5`
   - **Reason**: Increasing the spacing between rows enhanced coverage efficiency without sacrificing completeness.

---

## Methodology

The tuning process involved iterative testing and visualization using `rqt_plot` and `ros2 topic echo`. Each parameter was adjusted individually to observe its effect on performance metrics like cross-track error, velocity profiles, and cornering behavior.

### Challenges and Solutions

- **Oscillations in Linear Velocity**: Addressed by reducing `Kp_linear` and `Kd_linear`.
- **Imprecise Cornering**: Resolved by increasing `Kp_angular` and fine-tuning `Kd_angular`.
- **Row Overlap**: Improved by increasing the `spacing` parameter.

---

## Attachments

1. **Execution Videos:**

   Google Drive link to the videos of the entire execution: https://drive.google.com/drive/folders/1K2Ki4LxZDIjS1F74YU6j1IUDI9-FShzz?usp=sharing

---

## Future Work

- **Complete Extra Credit**: Implement steps to test and visualize the custom message integration fully.
- **Performance Optimization**: Further refine parameters to reduce the average cross-track error below 0.02 units.

---

## Conclusion

This project provided valuable insights into PD controller tuning and trajectory optimization. The implementation of a custom message type highlights the versatility of ROS2 and sets the stage for advanced performance monitoring in future assignments.

---

