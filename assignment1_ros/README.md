# Assignment1-ROS


```bash
    sudo apt install ros-melodic-effort-controllers
    rosrun xacro xacro -o snake.urdf snake.urdf.xacro
    check_urdf snake.urdf
    urdf_to_graphiz snake.urdf 
    chmod +x snake.py                                        
    roslaunch assignment1_ros rviz.launch model:=snake.urdf.xacro
    roslaunch assignment1_ros gazebo.launch model:=snake.urdf.xacro
    roslaunch assignment1_ros snake.launch
    cd ~/catkin_ws
    catkin_make && source devel/setup.zsh   
```
