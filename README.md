# Maze solver Robot With ROS.
The main idea of this project is to make the robot leave the maze, by detecting and following the wall. using the **turtle_bot** robot.
![image](https://user-images.githubusercontent.com/83847529/226198990-40f09279-8a79-4cf1-b6d5-f465cba1f506.png)

# steps of creating the project.
## 1. Creating the catkin directory.
First i start with creating ```catkin_ws``` directory which is my workspace. inside this directory i created ```src``` directory. which represent the **source** of my workspace.
can be created using the following command
```mkdir -p ~/catkin_ws/src```
## 2. Creating the ```wall_follower``` package.
Inside ```src``` direcotry i created a package called **wall_follower** package using the following command ```catkin_create_pkg wall_follower rospy```
this is the main package which contains the main files for the project.
## 3. Creating the world.
I used Gazebo builder to build the maze from the scratch and create ```.world``` file contains my robot and the maze, it saved to my package in the following directory.  ```/home/<user>/catkin_ws/src/wall_follower/worlds```, the world file called ```model.world``` .
![Screenshot 2023-03-17 222031](https://user-images.githubusercontent.com/83847529/226199796-2fa0c3c4-22f2-47be-8196-b493fd793799.png)

## 4. Creating the launch file.
I created manually the ```.launch```, to inculde the world and python codes supposed to be run directly after launch the project. 
the ```.launch``` is an XML format, can be found inside the ```wall_follower``` package in the following directory ```/home/<user>/catkin_ws/src/wall_follower/launch```, The launch file called ```maze_start.launch```.

``` XML
<launch>
<!-- My Package launch file -->
<node pkg="wall_follower" type="maze.py" name="MazeSolverNode" output="screen"> <!-- creating node named  MazeSolverNode to intiate the python code once the robot is lanuched-->
</node> 

<include file="$(find gazebo_ros)/launch/empty_world.launch"> <!-- This line to include the empty_world from gazebo for the logic perpose. -->
   <arg name="world_name" value="$(find wall_follower)/worlds/model.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
   <arg name="paused" value="false"/>
   <arg name="use_sim_time" value="true"/>
   <arg name="gui" value="true"/>
   <arg name="headless" value="false"/>
   <arg name="debug" value="false"/>
</include>

</launch>
```

## 5. Building my workspace.
then i built my **catkin_ws** workspace using the following commands.
```
catkin_init_workspace       #initiates the catkin workspace
cd ~/catkin_ws
catkin_make                 #compiles the catkin workspace
```

# How to run the project.
## 1. Cloning the ROS workspace.
first you have to clone the repository from git hub, or download the ```.zip``` file and unzip it on the ```home``` directory.
## 2. create workspace folders.
 in home directory create workspace and source folders using the following command 
```mkdir -p ~/catkin_ws/src```
## 3. creating the package.
after creating the workspace and source folder navigate to source folder to create the package using the following commands.
```
cd ~/catkin_ws/src/
catkin_create_pkg wall_follower rospy
```
## 4. moving the folders from repository.
after creating the package now let's move the ```launch``` folder,```world``` and ```scripts``` folders to our package directory ```/home/<user>/catkin_ws/src/wall_follower```

## 5. building our workspace.
Now the workspace is ready to be built, it can be built using the following commands.
```
cd ~/catkin_ws
catkin_init_workspace       #initiates the catkin workspace
catkin_make                 #compiles the catkin workspace
```

## 6. Sourcing the workspace.
Then you have to source the workspace using the following command.
``` source ~/catkin_ws/devel/setup.bash ```
## 7. Launching.
After sourcing the workspace all you have to do is to launch the package using the following command.
```roslaunch wall_follower start_maze.launch```
it supposed to open Gazebo and excute the python code with the world we created.

# Python Code Explaination.
In the ```wall_follower``` package, we can find in ```/home/<user>/catkin_ws/src/wall_follower/scripts``` directory two main python file ```pid.py``` and ```maze.py```.
## 1. pid.py.

First, the code defines a class called PID. When the __init__ method is called, it sets up the various parameters that will be used in the PID controller:

- kp, ki, and kd: These are the coefficients for the proportional, integral, and derivative terms of the controller, respectively. They determine how much weight is given to each term in the overall control output.
- outMin and outMax: These are the minimum and maximum possible output values for the controller. The output of the controller will be clamped to these values if it exceeds them.
- iMin and iMax: These are the minimum and maximum possible integral error values. The integral error is accumulated over time and can grow very large if not limited,    so these parameters prevent it from getting too large.

The resetValues method resets the state of the controller, setting the last error, sum error, and last time to zero.

The pidExecute method is the main method of the controller. It takes in two parameters: the desired value (should) and the actual value (actValue) that the controller is trying to control. It first calculates the current error between the desired and actual values. It then calculates the integral and derivative terms of the controller based on the current error and the time elapsed since the last calculation. Finally, it combines the proportional, integral, and derivative terms to calculate the output value for the controller.

The output value is then clamped to the minimum and maximum possible output values, and returned as the final output of the controller.
```output = kp * error + ki * sumError + kd * dError```.

``` python

def pidExecute(self, should, actValue):
    now = rospy.Time.now().to_nsec()
    timeChange = now - self.lastTime
    error = should - actValue
    newErrorSum = self.sumError + (error * timeChange)
```

The first few lines of the method calculate the current time, the elapsed time since the last calculation, and the current error between the desired and actual values. should is the desired value that the controller is trying to achieve, while actValue is the current actual value. The current error is simply the difference between these two values.

The sumError variable is used to keep track of the integral error over time. The integral error is the sum of all the past errors, and it is multiplied by the integral gain ki to produce the integral term of the controller. In the code above, newErrorSum is calculated by adding the current error multiplied by the elapsed time to the previous sumError value.

``` python

if((newErrorSum >= self.intMin) and (newErrorSum <= self.intMax)):
    self.sumError = newErrorSum
```

The if statement above checks if the new integral error is within the minimum and maximum limits specified by iMin and iMax. If it is, then sumError is updated to the new value. If it is not, sumError remains unchanged.

``` python
dError = (error - self.lastError) / timeChange
output = (self.kp * error) + (self.ki * self.sumError) + (self.kd * dError)
```

The next few lines of the method calculate the derivative term of the controller, as well as the overall output value. The derivative term is calculated by subtracting the last error from the current error, dividing by the elapsed time, and multiplying by the derivative gain kd. The overall output value is calculated by summing the proportional, integral, and derivative terms, each multiplied by their respective gains.
``` python
self.lastError = error
self.last = now
if(output > self.outMax):
    output = self.outMax
if(output < self.outMin):
    output = self.outMin
return output
```
Finally, the last error and last time are updated for the next iteration of the controller, and the output value is clamped to the minimum and maximum possible values specified by outMin and outMax. The output value is then returned as the final output of the controller.

## 2. maze.py.
The robot uses laser sensors for wall detection and following. The script starts by initializing ROS nodes, setting up subscribers and publishers, and setting some parameters, such as the distance to the wall and the minimum laser values for obstacle detection.

The ```startSolver``` function is the main function that runs the robot's maze solving algorithm. It first initializes the knownPoints list with the robot's current position. Then, it retrieves the minimum laser value in front of the robot for obstacle detection. The ```driveState``` variable stores the robot's current state, which is initially set to ```"WallDetection"```. The ```wallDetection``` function is then called to detect a wall for the robot to follow.

The ```wallDetection``` function rotates the robot by a specified angle and speed until it detects a wall using the laser sensors. Once a wall is detected, the ```driveState``` variable is set to ```"driveToWall"```. The robot then moves forward until it reaches the wall or detects an obstacle in front of it. If an obstacle is detected, the robot rotates again to search for a wall to follow. If the robot reaches the wall, it rotates to align itself with the wall and then starts wall following.

The ```wallFollower``` function is responsible for controlling the robot's movement while following the wall. It does so by using a PID controller to adjust the robot's angular velocity based on the distance from the wall detected by the laser sensors. The function also checks for loop detection by comparing the robot's current position with the knownPoints list. If the robot is detected to be in a loop, it sets the ```driveState``` variable back to ```"WallDetection"```.

The script continues to run in a loop until the ROS node is shut down. During each iteration, the laser and odom data are checked, and the robot's movement is updated based on its current state. The velocity commands are published to the ```/mobile_base/commands/velocity``` topic, and the robot moves accordingly.
