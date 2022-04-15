# drone_sim
A simple drone sim written in C (oh god)

How to use:

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
5. Blender_socket is used for linux systems while blender_socket_windows is for windows systems. The windows version requires Ws2_32.lib



