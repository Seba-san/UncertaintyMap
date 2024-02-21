#!/usr/bin/env python3
""" Toma los waypoints de un topic y luego los publica de forma sistematica en otro topic para que el agente realice la trayectoria."""


import rospy
from nav_msgs.msg  import Path
from std_msgs.msg import Float64 
from std_msgs.msg import Bool
from geometry_msgs.msg import Point as dot_command
from geometry_msgs.msg import PoseStamped
from GET_data import GET_data

class Manager:
    def __init__(self):
        self.dot_command_topic=rospy.Publisher('/dot_command',dot_command,queue_size=1)
        self.err_distance_pub=rospy.Subscriber('/err_distance',Float64,self.callback_err_distance,queue_size=1)
        self.path_incomming =rospy.Subscriber('/path_rrt_star',Path,callback=self.callback_path,queue_size=1)
        self.path_state=rospy.Publisher('/path_ready',Bool,queue_size=1) # True if the path is ready to be processed
        self.new_path=False
        self.err_distance=1
        
            
    def callback_path(self,msg=Path()):
        print('new path received')
        self.new_path=True
        self.path=msg.poses  
        #print(self.path)
        #import pdb; pdb.set_trace()
        
    def callback_err_distance(self,msg=Float64()):
        self.err_distance=msg.data      

if __name__=='__main__':  
    rospy.init_node('waypoint_manager')
    M=Manager()    
    rate=rospy.Rate(0.5)
    min_distance=0.4
    final_distance=0.8
    while not rospy.is_shutdown():
        if M.new_path:
            M.new_path=False
            M.path_state.publish(False)
            print('it is processing path')
            for idx in range(len(M.path)-1):                
                # hay que cambiar a que funcione todo con absoluto, viendo los datos del slam.
                # convertir de puntos absolutos a puntos relativos
               
                p0=M.path[idx]
                p1=M.path[idx+1]
                #p0=PoseStamped()
                #p1=PoseStamped()

                #dx=p1.pose.position.x-p0.pose.position.x
                #dy=p1.pose.position.y-p0.pose.position.y

                gt_data=GET_data()
                while not gt_data.got_states:
                    pass # wait to get the obstacle map
                
                pose_agent=gt_data.states[0:2]
                del gt_data
                dx=p1.pose.position.x-pose_agent[0]
                dy=p1.pose.position.y-pose_agent[1]

                print('current path: ',dx,' ',dy)
                dot_cmd=dot_command()
                dot_cmd.x=dx
                dot_cmd.y=dy                
                M.dot_command_topic.publish(dot_cmd)
                rate.sleep()
                if idx+2==len(M.path):
                   while M.err_distance>final_distance: 
                       rate.sleep()  

                   dot_cmd.x=0
                   dot_cmd.y=0  
                   M.dot_command_topic.publish(dot_cmd)

                else:
                   while M.err_distance>min_distance: 
                       rate.sleep()

                #while M.err_distance>min_distance: 
                #        rate.sleep()

                if M.new_path:
                    break
            print('Path finished')
                
        rate.sleep()
        M.path_state.publish(True)
    
    
 