#!/usr/bin/env python
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates as models


class Orientation:
    """ Get orientation of robot from gazebo
    
    Has a subscriber to gazebo model_states topic and calculate the orientation
    """
    def __init__(self):
        topic_robot_state='/gazebo/model_states'
        self.orientation_sub=rospy.Subscriber(topic_robot_state,models,self.get_orientation,queue_size=1)
        
    def get_orientation(self,data=models):
        """
        get orientation from gazebo and calculate rotation
        Input:
            .- gazebo model
        Output:
            .- self.orientation: Orientation of robot in radians
        """
        n_=len(data.name)
        robot_name='robot1'        
        if robot_name in data.name:
            for i in range(n_):
                if data.name[i]==robot_name:
                    break
        else:
            rospy.logerr('No se encontro el robot ' +robot_name+' dentro de los modelos de gazebo')        
            return
        qz=data.pose[i].orientation.z
        qw=data.pose[i].orientation.w
        self.calculate_orientation([qz,qw])
        
    def calculate_orientation(self,data):
        qz=data[0]
        qw=data[1]
        self.orientation=np.math.atan2(2*(qz*qw),1-2*(qz**2))