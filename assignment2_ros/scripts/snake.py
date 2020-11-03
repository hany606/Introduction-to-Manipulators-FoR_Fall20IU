#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from math import sin

num_joints = 5
max_joint_val = 2

# rostopic pub /snake/joint5_position_controller/command std_msgs/Float64 "data: 8"
rospy.init_node('auto_joint_controller', anonymous=True)

offsets = [0.01*i for i in range(num_joints)]

def joint_publisher(num):
    pub = rospy.Publisher("/snake/joint%d_position_controller/command"%(num), Float64, queue_size=10)
    return pub

def calculate_joint_pos(num, t):
    return sin((t+offsets[num])*7)*max_joint_val

rate = rospy.Rate(10) # 10hz
publishers = []

for i in range(num_joints):
    publishers.append(joint_publisher(i+1))

while not rospy.is_shutdown():
    for i in range(num_joints):
        pos_i = calculate_joint_pos(i, rospy.get_time())
        rospy.loginfo("Joint #%d pos: %f snake/joint%d_position_controller/command"%(i+1, pos_i, i+1))
        publishers[i].publish(data=pos_i)
        rate.sleep()
