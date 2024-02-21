#!/usr/bin/env python3
"""
This code is the core of active slam algorithm. It is a ROS node that administrates the whole process.

"""

import rospy
from std_msgs.msg import Float64 as debug
from std_msgs.msg import Float32 as divergence
from std_msgs.msg import Bool
from geometry_msgs.msg import Point as dot_command
import subprocess
import ast
import time
import sys
import numpy as np

def string_list_to_float_list(data):
    # chat g p t hizo esto
    try:
        # Utilizamos ast.literal_eval para evaluar la cadena de texto como una expresión de Python
        lista_datos = ast.literal_eval(data)
        return lista_datos
    except (ValueError, SyntaxError) as e:
        print("Error al convertir los datos:", e)
        return None


class Limmit_cycle_detector:
    def __init__(self):
        self.diver=0
        self.path=[]
        pass

    def divergence_analysis(self,divergence_predicted,path):
        if self.diver<divergence_predicted:
            self.diver=divergence_predicted
            self.path=path
            return False
        elif self.path==path:
            return True
            

class Administrator:
    """
    TODO: agregar que se pueda acceder al modo debug desde los argumentos
    """

    def __init__(self):        
        state_sub=rospy.Subscriber('/err_distance', debug, self.state_callback)
        state_sub2=rospy.Subscriber('/divergence', divergence, self.get_divergence)
        self.simulation_call=rospy.Publisher('/start_simulation', Bool, queue_size=1)
        #self.simulation_ends=rospy.Subscriber('/end_simulation', dot_command,callback=self.simulation_end_callback, queue_size=1)
        self.path_follower=rospy.Publisher('/dot_command', dot_command, queue_size=1)                                        
        self.begin_simulation=False
        self.begin_trajectory=False
        self.minimum_distance=1.5
        self.dot_command=dot_command()
        self.timeout=20#60

    #def simulation_end_callback(self,data=dot_command):
    #    self.begin_trajectory=True
    #    self.begin_simulation=False
    #    self.dot_command=data
    def get_divergence(self,data=divergence()):
        self.current_divergence=data.data

    def state_callback(self,data=debug()):
        if abs(data.data)<self.minimum_distance:
            self.begin_simulation=True
            self.begin_trajectory=False

    def run(self,debug_mode=0):
        rospy.init_node('active_slam_core', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        planner_mpc= rospy.get_param('planner_mpc')
        print('Planner MPC set to: ',planner_mpc)
        planner=rospy.get_param('planner')
        print('Planner mode set to: ',planner)
        uncertainty_frontier=rospy.get_param('UF')
        print('Uncertainty frontier considered:' ,uncertainty_frontier)
        cycle=Limmit_cycle_detector()
        divergence_predicted=0
        self.current_divergence=0

        while not rospy.is_shutdown():
            if self.begin_simulation:
                if divergence_predicted>self.current_divergence:
                    print("cuidado, hay incosistencias en el ciclo")
                    print("divergencia predecida: ",divergence_predicted, "divergencia obtenida: ",self.current_divergence)
                    print("diferencia de divergencias: ",self.current_divergence-divergence_predicted)
                else:
                    print("diferencia de divergencias: ",self.current_divergence-divergence_predicted)
              
                print('Starting simulation...')
                #time.sleep(5)
                #txt = subprocess.run(['python3', 'simulated_2d_slam2.py'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,text=True)
                #txt = subprocess.run(['python3', 'simulated_2d_slam.py','1'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,text=True)
                #import pdb; pdb.set_trace()
                #print(txt.stdout)               
               
                if debug_mode:
                    process = subprocess.Popen(['python3', 'simulated_2d_slam2.py','1'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                else:                             
                    process = subprocess.Popen(['python3', 'simulated_2d_slam2.py'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                

                # Esperar un tiempo máximo para la finalización del proceso
                timeout = self.timeout  # segundos
                start_time = time.time()
                while process.poll() is None:
                    elapsed_time = time.time() - start_time
                    if elapsed_time > timeout:
                        #process.terminate()
                        #txt = process.stdout.read()
                        #print(txt)
                        process.kill()
                        print('Simulation timed out')
                        break
                if process.returncode == 0:
                    txt = process.stdout.read()
                    #import pdb; pdb.set_trace()
                    data = txt.split('\n')[-2]
                    divergence_predicted_str=txt.split('\n')[-3]
                    #data_out = string_list_to_float_list(data)
                else:
                    print('Simulation failed with exit code:', process.returncode)
                    continue
                
                #data=txt.stdout.split('\n')[-2]
                if debug_mode:
                    print(txt)

                divergence_predicted=string_list_to_float_list(divergence_predicted_str)  
                data_out=string_list_to_float_list(data)
                print('divergencia predecida: ',divergence_predicted) 
                if cycle.divergence_analysis(divergence_predicted,data_out):
                    print('Simulation finalized due cycle limit detection')
                    rospy.set_param('ending_condition','limit cycle')
                    #break
                if divergence_predicted==-1.0: # TODO: cambiar esta forma de transmitir informacion.
                    print('Simulacion finalizada por falta de fronteras')
                    rospy.set_param('ending_condition','frontiers')
               


                data_out=string_list_to_float_list(data)
                
                if data_out[0][0]==0 and data_out[0][1]==0 and data_out[1][0]==0 and data_out[1][1]==0:
                    print('Simulation was completed')
                    print(data_out[0:2])
                    if divergence_predicted>0: # TODO: cambiar esta forma de transmitir informacion.
                        print('Simulacion finalizada por falta de movimiento')
                        rospy.set_param('ending_condition','still')
                    break                                    
                try:     
                    x=data_out[0][0]
                    y=data_out[0][1]
                    self.dot_command.x=x
                    self.dot_command.y=y
                    self.path_follower.publish(self.dot_command)
                    print('First New set point: ',x,y)
                    self.begin_simulation=False
                    self.minimum_distance=np.sqrt(x**2+y**2)-1
                    while not self.begin_simulation:
                        if rospy.is_shutdown():
                            break

                    if planner_mpc:
                        self.begin_simulation=False
                    else:
                        x=data_out[1][0]
                        y=data_out[1][1]
                        self.dot_command.x=x
                        self.dot_command.y=y
                        self.path_follower.publish(self.dot_command)
                        print('Second New set point: ',x,y)
                        self.begin_simulation=False


                #x=float(data.split(',')[0][1:])
                #y=float(data.split(',')[1][0:-1])
                #self.dot_command.x=x
                #self.dot_command.y=y
               ## self.path_follower.publish(self.dot_command)
                #print('New set point: ',x,y)
                #self.begin_simulation=False
                except:
                    print('Ocurrio un error, revisar el archivo simulated_2d_slam2.py')
                    import pdb; pdb.set_trace()

            rate.sleep()

if __name__=='__main__':
    admin=Administrator()
    if len(sys.argv)>1:
        admin.run(int(sys.argv[1]))
    else:
        admin.run()

    print('Ending condition: ',rospy.get_param('ending_condition'))
    
   
