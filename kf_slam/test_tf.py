#!/usr/bin python3
"""
Testea la estimaciones del kalman
"""

import tf2_ros
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
import numpy as np

def main():
    rospy.init_node('Inspector')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    ts=1
    rate = rospy.Rate(1/ts)
    last_angle=0
    radius=0.108
    last_pos=[0,0]
    while not rospy.is_shutdown():        
        try:
            trans = tfBuffer.lookup_transform('robot1_tf/base_link', 'robot1_tf/left_wheel', rospy.Time(0))    
            trans_world = tfBuffer.lookup_transform('robot1_tf/odom_groundtruth', 'robot1_tf/base_link', rospy.Time(0))         
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        #trans=TransformStamped()
        #trans.transform.translation.x
        trans=trans.transform
        

        #ang_=
        angles=euler_from_quaternion(quaternion=[trans.rotation.w,trans.rotation.x,trans.rotation.y,trans.rotation.z],axes='sxyz')
        
        ang_velocity=(angles[1]-last_angle)/ts
        lin_vel=ang_velocity*radius
        vel_x=(trans_world.transform.translation.x-last_pos[0])/ts
        vel_y=(trans_world.transform.translation.y-last_pos[1])/ts
        vel_=np.math.sqrt(vel_x**2+vel_y**2)
        print('angles: ',angles[1],'angular velocity: ',ang_velocity, 'linear velocity: ',lin_vel,'linear velocity 2: ',vel_)
        last_angle=angles[1]
        last_pos=[trans_world.transform.translation.x,trans_world.transform.translation.y]

        rate.sleep()


if __name__=='__main__':
    main()