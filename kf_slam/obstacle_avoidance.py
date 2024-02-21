#!/usr/bin/env python


from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np
import rospy
from several_classes import Orientation
import sys

class Avoid:
    """
    This class is used to avoid obstacles; Run with laser scanner data, and publish a new setpoint.
    
    No map is required, only laser scanner data.
    """
    def __init__(self,minimal_distance=None):
        # if minimal_distance is None:
            # minimal_distance=rospy.get_param('min_distance_obstacle_avoidance')
        # 
        # self.minimal_distance=minimal_distance
        # self.distance_new_setpoint=1.0
        # self.minimal_stop_distance=minimal_distance-0.1
        self.update_parameters()
        self.flag_working=False
        # Creamos un nodo de ROS llamado "avoidance_obstacles"
        rospy.init_node('avoidance_obstacles', anonymous=True)        
        self.ori=Orientation()
        rospy.loginfo('Nodo avoidance_obstacles creado con minimal_distance='+str(minimal_distance))

        # Nos suscribimos al topic "laser_scanner"
        rospy.Subscriber("/robot1/laser/scan_lidar_horizontal", LaserScan, self.callback_laser,queue_size=1)

        # Publicamos en el topic "command"

        self.pub_command = rospy.Publisher("/dot_command", Point, queue_size=1)

        # Mantenemos el nodo activo
        rospy.spin()
        
    def update_parameters(self):
        minimal_distance=rospy.get_param('min_distance_obstacle_avoidance')
        self.minimal_distance=minimal_distance
        self.distance_new_setpoint=0.7 #0.7
        self.minimal_stop_distance=minimal_distance-0.1

    def callback_laser(self, data=LaserScan()):

        self.update_parameters()
        # Esta función se ejecuta cada vez que se recibe un mensaje en el topic "laser_scanner"
        # data es el objeto que contiene los datos recibidos
    
        # Creamos una lista vacía para almacenar los puntos de obstáculos detectados
        #import pdb;pdb.set_trace()
        v_data=np.array(list(data.intensities))# Vector data
        ranges=np.array(list(data.ranges))
        obstacle_range=ranges[v_data>0.0]
        idx=obstacle_range<self.minimal_distance

        if not np.any(idx):
            self.flag_working=False
           # rospy.loginfo('bandera borrada')
            
            #rospy.logdebug_throttle_identical(msg='bandera borrada',period=1.0)
            return
        if self.flag_working:
            return
        #if np.any(obstacle_range<self.minimal_stop_distance):
        #    rospy.logerr('Obstaculo muy cercano, se detiene el robot')
        #    command=Point()
        #    command.x=0.0
        #    command.y=0.0
        #    self.pub_command.publish(command)
        #    return
        #ranges[np.isinf(ranges)]=5.0
        #ventana=100
        #media_movil=np.convolve(ranges, np.ones(ventana)/ventana, mode='valid')
        #idx=np.argmax(media_movil)+50
        idx=np.argmin(ranges)
        #import pdb; pdb.set_trace()

        

        # an obstacle was detected
        
        alpha=data.angle_min + idx * data.angle_increment
        
        
        r=self.distance_new_setpoint
        fi=self.ori.orientation
        #if self.flag_working and (abs(fi+alpha)<0.17 or abs(fi+alpha)-np.pi<0.17): #0.17 son 10 grados en radianes.
        #    return
        tita=fi+alpha
        #print(tita)
        new_setpoint=np.array([r*np.math.cos(tita+np.pi),r*np.math.sin(tita+np.pi)])
        #new_setpoint=np.array([r*np.math.cos(tita),r*np.math.sin(tita)])
        
        #obstacle_x=obstacle_range[idx]*np.math.cos(data.angle_min + np.arange(len(obstacle_range))[idx] * data.angle_increment)
        #obstacle_y=obstacle_range[idx]*np.math.sin(data.angle_min + np.arange(len(obstacle_range))[idx] * data.angle_increment)
        
        command=Point()
        command.x=new_setpoint[0]
        command.y=new_setpoint[1]
        # Publicamos el mensaje en el topic "command"
        self.pub_command.publish(command)
        rospy.logwarn("An obstacle was detected too close. New setpoint: "+str(new_setpoint))
        self.flag_working=True



if __name__ == '__main__':
    argv=sys.argv
    #import pdb;pdb.set_trace()
    try:
        Avoid()
        #if len(argv)>1: # Parece que vienen argumentos igual...
        #    minimal_distance=float(argv[1])
        #    Avoid(minimal_distance=minimal_distance)
        #else:
            
    except rospy.ROSInterruptException:
        pass
