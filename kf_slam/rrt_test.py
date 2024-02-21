#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import time
from GET_data import GET_data
import sys
import numpy as np

if __name__=='__main__':
    if len(sys.argv)!=3:
        print('Error: se deben ingresar dos argumentos, x e y del goal')
        sys.exit()
    rospy.init_node('rrt_test_node')
    pub_=rospy.Publisher('/planning_route_rrt_star',PoseArray,queue_size=1)
    gt_data=GET_data()
    while not gt_data.got_states:
        pass # wait to get the obstacle map
    
    pose_agent=gt_data.states[0:2]
    P=gt_data.P
    del gt_data
    D=np.array([[P[0][0],P[0][1]],[P[1][0],P[1][1]]])
    sigma_robot=np.power(np.linalg.det(D),0.25)
    #cost_robot=sigma_robot/(0.1*0.01) # cell_size*Q
    #time.sleep(5)
    #import pdb; pdb.set_trace()
    mgs1=PoseArray()
    mgs1.header.seq=11
   
    # Start
    p1=Pose()
    p1.position.x=pose_agent[0] # x e y estan invertidos en el mapa
    p1.position.y=pose_agent[1]
    p1.position.z=sigma_robot
    mgs1.poses.append(p1)
    # Goal
    p2=Pose()
    p2.position.x=float(sys.argv[1])
    p2.position.y=float(sys.argv[2])
    mgs1.poses.append(p2)
    print(mgs1.poses)
    pub_.publish(mgs1)
    rospy.signal_shutdown('nodo test rrt star finalizado.')
            
    #rete=rospy.Rate(2)
    ## No se xq no funciona si no le pongo este while...
    #while not rospy.is_shutdown():
    #    print('Enviando...')
    #    pub_.publish(mgs1)
    #    
    #    rete.sleep()
#
    #    
    #
    #rospy.signal_shutdown('nodo test rrt star finalizado.')