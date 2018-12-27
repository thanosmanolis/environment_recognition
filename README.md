# environment_recognition


## Description

Take input from LIDAR and RGBD Camera, and use it to recognize the type of environment we are in. Decide the environment's type from some specific factors, such as features/no features, density, darkness, doors, indoors/outdoors, corridors.


## Description of each procedure

In general, node "recognition" subscribes to the /scan topic, and publishes the environmental type at the /environment topic. Also during the whole procedure, it prints the values of some useful variables, so that we can check on the same time if the result is the expected one. In detail:

### laserCallBack()
- Compare every single value of the *ranges*[] list with its previous one, starting from *ranges*[1].
  - If the current element is out of the scope of +-0.35*(previous element), raise the variable that counts the variances of each scan by 1 (*temp_var_*). This means that the continuity has stopped. Thus, the robot probably sees the start/end of a feature and apparently not a wall. (The do nothing command is executed, so that it doesn't count it as a feature, if both elements are infinite.)
  - For every element, check if it has infinite value. If yes, raise infinite counter (*counter_inf_*) by 1.
- Check if all *ranges*[] are infinite.
  - If yes, then the robot sees nothing.
  - If not, then give a value to the variable that holds the sum of all the features' sizes the robot sees at the current scan (*temp_features_*). Also, call the function to calculate the average values of the last 100 scans (*averageValues*()).

### averageValues()
- For the first 100 scans, raise the samples_ by 1. Then, keep it to 100, so that we always calculate the average value of the last 100 scans. Also, reset the index of the lists (*index_*), everytime it reaches 100.
- At each scan, throw away the value we added 101 scans before, and add the current one. In that way, we always get the average value from the last 100 scans, without having it reseted. 
- At last, check the environmental type, according to the information received from the whole procedure. We end up with the following variables, which are going to be our metrics for the environmental type decision:

  | Variable | Content |	  
  | --- | --- |
  | average_var_ | At each measurment, the robot sees a specific number of variances (big distance between 2 laser ranges). This variable holds the mean value of the variances detected at the last 100 scans. |
  | density_ | At each measurment, the robot sees a specific number of non-infinite values (and therefore it sees features). This variable holds the percentage of the mean value of non-infinite values detected at the last 100 scans (out of all the ranges in each scan, which is the value of *ranges_size_*). |
  | threshold_variance_ | A threshold we set, in order for the system to decide if it detects features, or just walls (i.e. in a maze). |
  | threshold_density_  | A threshold we set, in order for the system to decide if it is in a dense or a sparse environment. |			  
### Decision tree
1. **average_var_ <= threshold_variance_ AND density_ <= threshold_density_**

   There are just walls. For example, the robot could be in a maze. Also, it sees a small amount of walls, therefore the environment is sparse.

2. **average_var_ <= threshold_variance_ AND density_ > threshold_density_**

   There are just walls. For example, the robot could be in a maze. Also, it sees a large amount of walls, therefore the environment is dense.

3. **average_var_ > threshold_variance_ AND density_ <= threshold_density_**

   There are for sure some features. Probably, there are also walls. Also, it sees a small amount of features, therefore the environment is sparse.

4. **average_var_ > threshold_variance_ AND density_ > threshold_density_**

   There are for sure some features. Probably, there are also walls. Also, it sees a large amount of features, therefore the environment is dense.

## How to run it

1. Launch a turtlebot_gazebo world
2. Launch turtlebot_teleop or a navigation algorithm
3. Echo the topic /environment
4. Launch recognition.launch (change the parameters' values if needed)