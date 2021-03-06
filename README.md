# drone_sim
A simple drone sim written in C (oh god)

Features:
- Euler angle based Extended Kalman filter for state estimation.
- Discrete PID controller with derivative filter and anti-windup
- Sensor models with normally distributed noise using box-muller transform.
- Includes rotor dynamics such as blade flapping and induced drag.
- Numerical integration of the continuous dynamics using RK4.
- Waypoint tracking of random waypoints.
- No dynamic allocation (The implementation runs on average 0.5 ms on a STM32F401).
- Real-time visualisation of the simulation using blender.
- Option for quaternion based attitude controller (ETH).

This library is a simple educational example of a quadrotor autopilot and is not meant for real use. Although, the autopilot has been implemented and flight tested on a STEVAL-Drone. A repository of the flight proven code will be available in the future.

This project is still under development. Todo list:
- Implement MEKF
- Replace LU decomposition with Cholesky decomposition
- Implement MPC-based controller

The drone parameters are taken from:

Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems, Third Edition. Brian L. Stevens, Frank L. Lewis and Eric N. Johnson.

# How to use:

1. Run the code
2. Open the blend file and run the script.
3. Run the quadrotor operator (see the video).
   - Make sure that Developer Extras is enabled under Settings->Interface
   - Make sure space bar action is set to search under Settings->Keymap
   - Press space and write quadrotor, the operator should be top of the lsit.
		
		
Caveats:

1. The sim needs to be running before you run the quadrotor script in blender or else it will crash.
2. When the quadrotor operator is running you can left mouse click to change the camera angle.
3. To stop the quadrotor operator press escape or right mouse click.
4. Make sure the quadrotor operator is not already running before starting it again. Otherwise two of them will run concurrenlty and things will be messed up.
5. Blender_socket is used for linux systems while blender_socket_windows is for windows systems (project is set up for linux). The windows version requires ws2_32.lib

# Demonstration

https://user-images.githubusercontent.com/64291228/163497745-d44a8c4c-b1a1-4e43-90a9-74f91386d28e.mp4

