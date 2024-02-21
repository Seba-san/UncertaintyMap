#!/usr/bin/env python3

import rospy
import time
from scipy.io import savemat
from simulated_2d_slam import GET_data
from nav_msgs.msg import Path
import numpy as np
import sys

import argparse

class Path_subscriber:
    def __init__(self):
        self.slam_flag=False
        self.chasis_flag=False
        #rospy.init_node('path_subscriber_node', anonymous=True)

        # Suscribirse al topic 'path_chassis' con el tipo de mensaje nav_msgs/Path
        rospy.Subscriber('path_chassis', Path, self.callback_chassis)

        # Suscribirse al topic '/path_slam' con el tipo de mensaje nav_msgs/Path
        rospy.Subscriber('/path_slam', Path, self.callback_slam)

         # Mantener el nodo activo

        while not self.slam_flag or not self.chasis_flag:
            print('waiting for path data')
            time.sleep(0.3)

    def callback_chassis(self,msg):
        rospy.loginfo("Received path from path_chassis")
        self.path_chassis=msg
        self.chasis_flag=True       

    def callback_slam(self,msg):
        rospy.loginfo("Received path from slam")
        self.path_slam=msg
        self.slam_flag=True

    def path_to_array(self,msg):
        return np.array([(pose.pose.position.x, pose.pose.position.y, pose.header.stamp.to_sec()) for pose in msg.poses])

class Save_map:
    """ Catch data from slam and map servers and simulate posible movements
    
    This class depends on Get_data class and SLAM2D_simulated class.
    You can define the step of each movement, and then use a external function
    called make_control_signals to generate a list of control signals in a
    particular way. 
    The principal method is instances_slam2d_simulated that will generate a
    control action for each posible control signal and then make an instance
    of SLAM2D_simulated class. Each instance is a simulation of movements and
    get the divergence of  full map.
    
    TODO: in instances_slam2d_simulated method, do each one with a thread.
    Another posible way is make this class in golang and use goroutines.
    Maybe is better to use c++. For this  version each simulation in
    sim_.run() expend 1 second aprox.
    
    """
    def __init__(self,name='matlab_matrix',debug=0):
        """if debug==1 print divergence, if debug==2 publish a map and take its time..."""

        admin=GET_data()
        while not (admin.got_states and admin.exp_map.flag_  and admin.obstacle_map.flag_): # wait for a states data
            print('waiting for data')
            print('states: ',admin.got_states, 'exp_map: ',admin.exp_map.flag_,'obstacle_map: ',admin.obstacle_map.flag_)            
            time.sleep(1)
       #
        rospy.loginfo('data captured')
        self.debug=debug
        self.states,self.P,self.exp_map,self.obstacle_map=admin.get_data() 

        #rospy.signal_shutdown('now')
        path=Path_subscriber()
        self.path_chassis=path.path_to_array(path.path_chassis)
        self.path_slam=path.path_to_array(path.path_slam)

        D={}

        D['planner']=rospy.get_param('planner')
        D['sigma_max']=rospy.get_param('sigma_max')
        D['cell_size']=rospy.get_param('cell_size')
        D['initial_pose']=rospy.get_param('initial_pose')
        D['UF']=rospy.get_param('UF')
        D['uf_treshold']=rospy.get_param('uf_treshold')
        D['planner_mpc']=rospy.get_param('planner_mpc')
        D['map']=rospy.get_param('map')
        #D['ending_condition']=rospy.get_param('ending_condition')

        savemat(name+".mat", {'exp_map':self.exp_map, 'states':self.states,'P':self.P,'obstacle_map':self.obstacle_map,
                                      'path_chassis':self.path_chassis,'path_slam':self.path_slam,'Data':D})       
        print('Mapa guardado como: ',name+'.mat')
            

def main():
    parser = argparse.ArgumentParser(description='Guarda todos los parametros de la simulacion en un archivo .mat, sumado a los mapas, trayectorias y divergencia')

    # Agrega argumentos necesarios para tu programa
    parser.add_argument('nombre', nargs='?', default=None, help='Nombre para Save_map')

    args = parser.parse_args()

    if args.nombre is None:
        sim = Save_map()
    else:
        sim = Save_map(name=args.nombre)


if __name__=='__main__':
    main()

    
          

        
    


    