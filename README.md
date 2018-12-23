# environment_recognition

**************************
*    Repo description    *
**************************

Take input from LIDAR and RGBD Camera, and use it to recognize the type of environment we are in. 
Decide the environment's type from some specific factors, such as features/no features, density, darkness, indoors/outdoors, doors, corridors.

***************************************
*    Description of each procedure    *
***************************************

In general, node "recognition" subscribes to the /scan topic, and publishes the environmental type at the 
/environment topic. Also during the whole procedure, it prints the values of some useful variables, so
that we can check on the same time if the result is the expected one. In detail:

------------------------------------------- laserCallBack() -------------------------------------------
- Compare every single value of the ranges[] list with its previous one, starting from ranges[1].
  - If the current element is out of the scope of +-0.35*(previous element), raise the variable
		that counts the variances of each scan by 1 (temp_var_). This means that the continuity has 
		stopped.Thus, the robot probably sees the start/end of a feature and apparently not a wall.
		(The do nothing command is executed, so that it doesn't count it as a feature, if both elements 
		are infinite.)
  - For every element, check if it has infinite value. If yes, raise infinite counter by 1.
- Check if all ranges[] are infinite.
	- If yes, then the robot sees nothing.
	- If not, then give a value to the variable that holds the sum of all the features' sizes 
		the robot sees at the current scan. Also, call the function to calculate the average 
		values of the last 100 scans.
------------------------------------------- averageValues() -------------------------------------------
- For the first 100 scans, raise the samples_ by 1. Then, keep it to 100 (so that we always) calculate
	the average value of the last 100 scans. Also, reset the index of the lists (index_), everytime it
	reaches 100.
- At each scan, we throw away the value we added 101 scans before, and add the current one. In that
	way, we always get teh average value from the last 100 scans, without having it reseted. 
- Lastly, we check how dense the scans are, and if the robot sees features, or if it is in a place
	which only has walls (or perhaps it's a maze). The result occurs from the threshold we set at the
	beginning.

***********************
*    How to run it    *
***********************

1. Launch a turtlebot_gazebo world
2. Launch turtlebot_teleop or a navigation algorithm
3. Echo the topic /environment
4. Launch recognition.launch (change the parameters' values if needed)