#!/usr/bin/env python
"""
El objetivo de este codigo es implementar un PID en simulacion. Para usarlo hay
que hacer:
python control.py /left_dc_motor/command /left_dc_motor/velocity /rrbot/left_wheel_controller/command
python control.py /right_dc_motor/command /right_dc_motor/velocity /rrbot/right_wheel_controller/command

El orden es el siguiente
python control.py control lectura set_point

El valor del set_point es en velocidad angular.

Inicialmente este controlador estaba en el el package my_controller.

Tiene 2 grandes bloques: 
    .- Una clase PID.
    .- Una clase que administra el controlador. ("Best_controller")

.- Clase PID:
    Es muy parecida a la implementada en el firmware de Arduino. 
.- Best_controller:
    Se encarga de escuchar los topics de set_point y de medicion para generar
    una accion de control. Tambien se puede gestionar algun procesamiento para
    estimar los estados a partir de las mediciones.

Posibles Mejoras:
    .- Actualizar los PID online.

"""
import rospy
from std_msgs.msg import Float32 as Control_command
from std_msgs.msg import Float64 
from gazebo_msgs.msg import LinkStates
import sys
import math
from pid import PID 
from sensor_msgs.msg import JointState 

class Best_controller:
    """
    Este controlador, tiene que escuchar /gazebo/joint_links y cada vez que le
    llege algo nuevo, tiene que publicar el valor del controlador.
    Acutualizacion v2:
        .- En lugar de escuchar /gazebo/link_sates, escucha el topic de los
        motores que se asigna cuando se llama este programa.
        
    Orden de los topics:
    actuador, sensor, set_point
    
    Si el patron a medir es velocidad, es muy ruidosa y por eso se le mete una media movil. 
    Esto genera un retardo en la respuesta del controlador.
    """
    def __init__(self,valores):
        self.name_space=rospy.get_namespace()
        self.name_space=self.name_space[0:-1]
        self.get_name_joints(valores)
        self.sensor=rospy.Subscriber(self.topic_sensor,JointState,self.process_sensor,queue_size=1)
        # send commands
        self.publisher=rospy.Publisher(self.name_effort_interface,Control_command,queue_size=5)
        
        #self.publisher_mean=rospy.Publisher(self.name_set_point+'_mean',Control_command,queue_size=5)
        self.command=Control_command()
        # recibe link_state
        #self.send_command=rospy.Subscriber(self.name_link_state,LinkStates,self.callback_controller)
        #self.send_command=rospy.Subscriber(self.topic_sensor,Control_command,self.get_measure)
        # recibe set point
        self.set_point_sub=rospy.Subscriber(self.name_set_point,Float64,self.get_set_point)
        # Configuracion de parametros del PID online
        #self.get_pid_parameters=rospy.Subscriber(self.pid_parameters,Float64,self.get_pid_parameters_callback)
        tmp=self.name_effort_interface
        tmp=tmp.split('/')
        self.node_name=tmp[2]+'/'

        self.PID=PID()
        self.PID.set_parameters(Kp=1.0 , Ki=1.0, Kd=0.0,Ts=0.01)
        if not rospy.has_param(self.node_name+'gains'):
            rospy.set_param(self.node_name+'gains', {'kp': 1.0, 'ki':1.0, 'kd': 0.0})

        self.set_point=0.0
        self.measure=0
        self.vel=0
        self.pos=[0 for i in range(2)]
        
        #self.publisher_debug=rospy.Publisher(self.node_name+'error',Control_command,queue_size=1)
        
    def process_sensor(self,data=JointState):
        # Mide son ruido
        #  TODO: como es un controlador de posicion, lee la posicion del joint, pero tendria que ser parte de la config
        m=data.velocity[0] # angular velocity
        #m=m*0.108
        self.measure=self.measure*9/10+m/10

    def get_set_point(self,data=Float64()):
        self.set_point=data.data

    def get_measure(self,data=Control_command()):
        #self.measure=self.measure*9/10+data.data/10
        self.measure=data.data
        #if abs(self.measure-data.data)>0.07: # Elimina los ruidos metiendo un umbral
        #    self.measure=data.data

    def compute_u(self):
        # Compute and send command signal 
        kp=rospy.get_param(self.node_name+'gains/kp')
        ki=rospy.get_param(self.node_name+'gains/ki')
        kd=rospy.get_param(self.node_name+'gains/kd')
        self.PID.set_parameters(Kp=kp , Ki=ki, Kd=kd,Ts=0.01)
        self.command.data=self.PID.run(self.set_point,self.measure)
        self.vel=self.PID.y[0]

    def get_name_joints(self,valores):
        # Comando
        #self.name_effort_interface='/rrbot/left_wheel_controller/command'
        self.name_effort_interface=self.name_space + valores[0]
        # Leectura sensor
        #self.name_to_control='pioneer2dx::left_wheel'
        #self.name_to_control=valores[1]
        #self.name_link_state='/gazebo/link_states/'
        self.topic_sensor=self.name_space +valores[1]
        
        self.name_index=-1
        # Lectura set point
        self.name_set_point=self.name_space +valores[2]

        # parametros pid
        #self.pid_parameters=self.name_space+'/pid_parameters'
        
    def run(self):
        self.time=self.PID.Ts
        rospy.init_node('controller_python',anonymous=True)
        self.rate = rospy.Rate(1/self.time) # 10hz
        while not rospy.is_shutdown():
            self.compute_u()
            self.publisher.publish(self.command)
            #self.publisher_debug.publish(Control_command(self.measure))
            #self.publisher_mean.publish(self.vel)
            #print('comando:',self.command.data)
            #print('medido:',self.vel)
            #print('medido:',self.PID.y_sp[0])
            #print('medido:',self.PID.y[0])
            #print('medido:',self.PID.Normalizacion)
            
            self.rate.sleep()
    
def main(valores):
    control=Best_controller(valores)
    control.run()

if __name__=='__main__':
    valores=sys.argv[1:4]
    print(valores)
    
    try:
        main(valores)
    except rospy.ROSInterruptException:
        pass
