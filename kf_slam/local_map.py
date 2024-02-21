
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray as states_
from std_msgs.msg import MultiArrayDimension 
from std_msgs.msg import Float32 as divergence_msg
from scipy.stats import multivariate_normal as norm_distribution #https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.multivariate_normal.html
import scipy
from nav_msgs.msg import OccupancyGrid as map_msg
from copy import deepcopy as copy
#import time
import tf2_ros
import geometry_msgs.msg
import sys


class Local_map:
    """ This class create a local map from the global map for each instance of simulation
    """
    def __init__(self,states_cova_topic='/slam/states_cova',center_frame='robot1_tf/odom_groundtruth',topic_map='/slam/landmark_map',debug=False):
        self.debug=debug
        if debug:            
            self.sub=rospy.Subscriber(states_cova_topic,states_,self.get_states)            
            self.map_topic=rospy.Publisher(topic_map,map_msg,queue_size=1)
            self.publish_divergence=rospy.Publisher('/divergence',divergence_msg,queue_size=1)
            
        self.cell_size=0.1# square cells
        self.map_size=[int(60/self.cell_size),int(60/self.cell_size)]
        self.alpha=0.0001
        self.landmarks_map=np.full(shape=self.map_size,fill_value=self.alpha)
        self.full_map=np.full(shape=self.map_size,fill_value=self.alpha)

        self.exploration_map=np.full(shape=self.map_size,fill_value=self.alpha)
        self.exploration_map_bool=np.full(shape=self.map_size,fill_value=False)

        self.new_data=False

        self.FOV=5#m circular
        self.states=np.array([0,0])
        self.make_mask_FOV()
        self.new_data_=False
        
        self.center_frame=center_frame
        # For planification propuses

        pass


if __name__=='__main__':
    pass