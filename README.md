# Particle filter localization
particle filter for robot localization developed by Python programming language in a ROS melodic workspcace.

first of all add these in your ROS melodic or above workspace.
you can delete devel and build directories and enter `catkin_make` to make this application.

after that in a terminal enter:
`roscore`
then in a new terminal enter:

```
source devel/setup.bash
rosrun particle_filter particle.py
```
after a window containing robot, landmarks and particles appear, open a new terminal and enter codes below:
```
source devel/setup.bash
rosrun particle_filter move.py
```
now you can move the robot with arrow keys and see the convergence of particles to the robot position. 
 
