#!/usr/bin/env python3
"""La idea de esta clase es levantar de forma estandarizada topics del sistema ASLAM. Ademas NO GENERA SU PROPIO NODO"""

from std_msgs.msg import Float32MultiArray as states_
import rospy
import numpy as np

class MAP_:
    """ Generic implementation of a map
    elements:
    .- map_: map as a list of cells
    .- flag_: flag to know if the map is updated or not
    """
    def __init__(self):
        self.map_=[]
        self.flag_=False

class GET_data:
    """ This class is for get data from main slam and main map and save it in variables
        
    This class listen:
    .- /slam/states_cova topic 
    .- /exp_map topic 
    .- /obstacle_map_matrix topic
    Save this data in the follow variables:
    
    .- self.states: states vector
    .- self.P: covariance matrix
    .- self.exp_map: exploration map
    .- self.obstacle_map: obstacle map
    
    For this class to work, you need to publish once the topics /slam/states_cova and /exp_map, however
    it commente the "if self.got_states" and "if self.got_exp_map" in the callback functions you can listen 
    all the time the topics.
    """
    
    def __init__(self):
        self.exp_map=MAP_()
        self.obstacle_map=MAP_()
        self.got_states=False

        self.data_source='/slam/states_cova'
        self.data_source_topic=rospy.Subscriber(self.data_source,states_,callback=self.states_callback,queue_size=1)
        exploration_map_topic='/exp_map'
        self.data_source_topic=rospy.Subscriber(exploration_map_topic,states_,callback=self.get_map_callback,
                                                callback_args=self.exp_map,queue_size=1)
        obstacle_map_topic='/obstacle_map_matrix'
        self.data_source_topic=rospy.Subscriber(obstacle_map_topic,states_,callback=self.get_map_callback,
                                                callback_args=self.obstacle_map,queue_size=1)
        return
                
        
    def states_callback(self,data=states_()):
        """ Build states and covariance matrix P from Float32MultiArray topic
        First data is a state vector and the follow one is an covariance matrix
        """
        if self.got_states:
            return # only enter once
        N=data.layout.dim[0].size
        self.P=[]
        self.states=list(data.data[0:N])
        for i in range(N):
            i_0=(i+1)*N
            i_1=(i+2)*N
            self.P.append(list(data.data[i_0:i_1]))

        self.got_states=True

    def get_data(self):
        """ Return states, covariance matrix and exp_map
        """
        return self.states,self.P,self.exp_map.map_,self.obstacle_map.map_
        
    def get_map_callback(self,data,map_data=MAP_()):
        """ recibe from map server some map and save it in output_ variable
        Get Float32Multiarray message.
        This function was checked in test.py for little arrays
        """
        if map_data.flag_:
            return # only enter once
        s=[data.layout.dim[0].size,data.layout.dim[1].size ]
        map_=[]
        
        for k in range(s[0]):
            map_.append(list(data.data[k*s[1]:(k+1)*s[1]]))
        
        map_data.map_=np.array(map_) 
        map_data.flag_=True
        #print('argumento: ',arg)
