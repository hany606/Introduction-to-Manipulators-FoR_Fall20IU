# Assignment2-ROS

This package is build on ROS-Melodic. This package has a urdf for a crawling robot (Snake inspired). This crawling robot is similar to serial manipulators without base in the way of construction.

![Robot visualization](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment2_ros/imgs/snake_gazebo2.gif)
![Robot visualization](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment2_ros/imgs/snake_gazebo1.gif)
![Robot visualization](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment2_ros/imgs/snake_rviz.gif)

The initial robot has 5 revolute joints and their position controllers. The robot's joints and link can be extended by modifing ```snake.urdf.xacro``` and adding ```<xacro:extra_link num="idx" parent="idx_parent"/>```

![Robot visualization](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment2_ros/imgs/rqt_nodes_graph.png)

The robot is controlled with position controllers, and there is a node that publish to the topics of the controllers with values that is from scaled and shifted sin function in order to perform an oscilation move to make the snake move. The advanced way to control the snake inspired robots is by using CPG (Central Pattern Generator) that is one of the methods that was inspired from animals in both invertebrate and vertebrateanimals for  locomotion  control, it is widely used in Crawling locomotion and can be used here for serpentine gait for snake-inspired robot. Also, there is Serpenoid Curve Control function, this function is used to generatethe joint rythm in order to control the snake-inspiredrobot to perform serpentine locomotion.

## Requirements:

If ros-melodic-effort-controllers is not installed, you should install it:

```bash
    sudo apt install ros-melodic-effort-controllers
```

## How to use it?

* To run only the robot in rviz
```bash
    roslaunch assignment1_ros rviz.launch model:=snake.urdf.xacro
```

* To run only the robot in Gazebo without controller
```bash
    roslaunch assignment1_ros gazebo.launch model:=snake.urdf.xacro
```

* To run the robot in rviz and Gazeob and use the poition controller
```bash
    roslaunch assignment1_ros snake.launch
```


## Notes:

* In order to use nodes, in python you should make the files executable:

```bash
    chmod +x snake.py
```

* To clone the repo in different directory thann catkin_ws, you should link it to catkin_ws in order to compile the project:

```bash
    ln -s ~/repos/FoR_Fall20IU/assignment2_ros ~/catkin_ws/src/assignment2_ros
```

* In order to generate urdf tree from xacro files:

```bash
    rosrun xacro xacro -o snake.urdf snake.urdf.xacro
    check_urdf snake.urdf
    urdf_to_graphiz snake.urdf
    xdg-open snake.pdf
```

* In order to compile and make the catkin_ws:
  
```bash

    cd ~/catkin_ws
    catkin_make && source devel/setup.zsh   
```
