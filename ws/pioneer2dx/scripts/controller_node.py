#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist,Pose
from std_msgs.msg import Float64 as Control_command
#from std_msgs.msg import String
import threading
import sys, select, termios, tty # Esto es para leer el teclado
import os
from gazebo_msgs.srv import SetModelState, DeleteModel
from gazebo_msgs.msg import ModelState
from controller_manager_msgs.srv import LoadController, SwitchController
import math
import numpy as np

class ControlAction():
    """
    Esta clase genera un nodo para controlar:
    diff_drive_controller/DiffDriveController
    """
    def __init__(self,name):
        self.name=name
        self.getNameTopic()
        self.setPoint=Twist()
        self.keyBoardOperation()
        msg='Recordar que es necesario cargar el controlador antes' \
            ' en el launch file, ver ejemplo launch/controller.launch'
        print(msg)
        #self.beginController()
    #self.x = threading.Thread(target=self.controlAction)
        #self.controlAction()
        self.automaticMode=0 # Es para generar senales automaticas

    def getNameTopic(self):
        # Por ahora lo pongo asi. A futuro buscar una forma de automatizarlo
        #self._name='/rrbot/mobile_base_controller/cmd_vel'
        name_space='/robot1'
        self.left_wheel=name_space+'/left_wheel_controller/command'
        self.right_wheel=name_space+'/right_wheel_controller/command'
        
    def run(self):
        #name=getNameTopic()
        self.pub_l = rospy.Publisher(self.left_wheel, Control_command, queue_size=10)
        self.pub_r = rospy.Publisher(self.right_wheel, Control_command, queue_size=10)
        rospy.init_node(self.name, anonymous=True)
        self.time=0.1
        self.rate = rospy.Rate(1/self.time) #10hz
        #setPoint=Twist()
        print('Iniciando nodo')
        while not rospy.is_shutdown() and self.on:            
            #hello_str = "hello world %s" % rospy.get_time()            
            #rospy.loginfo(hello_str)
            self.getCommand()
        
        # genera un setPoint automatico
            if self.automaticMode!=0:
                self.automaticSetPoint()
                

            #self.pub.publish(self.setPoint)
            self.sP2_vel()
            self.pub_l.publish(self.wl)
            self.pub_r.publish(self.wr)
            
            self.rate.sleep()
        rospy.signal_shutdown('Nodo finalizado por el usuario')

    def _setPoint(self,acc=[0,0]):
        self.setPoint.linear.x+=acc[0]
        self.setPoint.angular.z+=acc[1]
        
        if self.setPoint.linear.x**2<0.0001:
            #print('set 0 lineal')
            self.setPoint.linear.x=0
        
        if self.setPoint.angular.z**2<0.0001:
            #print('set 0 angular')
            self.setPoint.angular.z=0
        
        print('linear: ',self.setPoint.linear.x,' angular: ', self.setPoint.angular.z)

    def sP2_vel(self):
        # Convierte las velocidades del vehiculo a las angulares de las ruedas
        x=self.setPoint.linear.x
        z=self.setPoint.angular.z
        # hay que obtener los paramertos
        r1,r2=0.108,0.108 # radio de las ruedas
        l= 0.34 # distancia entre las ruedas
        self.wl=x/r1-z*l/(2*r1)
        self.wr=x/r2+z*l/(2*r2)

    def killNode(self):
        # No se como se hace eso
        #rospy.signal_shutdown(reason)
        print('Cerrando nodo')
        self.on=False

    def getCommand(self):
        """
        Lee la funcion del diccionario moveBindings. Luego verifica si tiene
        argumentos y dependiendo de eso llama a la funcion definida en
        keyBoardOperation con o sin argumentos.
        """
        key=self.getKey()
        if key in self.moveBindings.keys():
            funcExe,_=self.moveBindings.get(key)
            if self.moveBindings[key][1]!=0:
                self.out=funcExe(self.moveBindings[key][1])
            else:
                self.out=funcExe()

            if self.out==-1:
               self.on=False
    
    def getKey(self):

        # Fuente principal: https://github.com/turtlebot/turtlebot/blob/melodic/turtlebot_teleop/scripts/turtlebot_teleop_key
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], self.time)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def keyBoardOperation(self):
        step=0.1
        self.moveBindings = {
            'a':[self._setPoint,[0,step*0.3]],
            's':[self._setPoint,[-step,0]],
            'd':[self._setPoint,[0,-step*0.3]],
            'w':[self._setPoint,[step,0]],
            'q':[self.killNode,0],
            'x':[self.setIniciar,0],
            'r':[self.respawnModel,0],
                '8':[self.setAutomaticMode,'8'],
                'o':[self.setAutomaticMode,'o']
               }
        self.on=True
    
    def setIniciar(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        #modelState=ModelState()
        setEstadoFun=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        modelState=ModelState()
        modelState.model_name='robot1'
        #pp=Pose()
        #pp.
        #pp.position.y=1
        modelState.pose.position.z=1
        resp=setEstadoFun(modelState)
        print('set in xyz=[0,0,1], rpy=[0,0,0]')

    def respawnModel(self):
        print('Inicializando Respawn model')
        service='/gazebo/delete_model'
        rospy.wait_for_service(service)
        deleteModel=rospy.ServiceProxy(service,DeleteModel)
        #deleteModel(model_name='pioneer2dx')
        deleteModel(model_name='robot1')
        #os.system("roslaunch pioneer2dx respawn.launch")
        os.system("roslaunch pioneer2dx multiple_robots.launch")
        #self.beginController()
        print('Respawn Finalizado')


    def beginController(self):
        print('configurando controlador')
        service='/rrbot/controller_manager/load_controller'
        rospy.wait_for_service(service)
        loadController=rospy.ServiceProxy(service,LoadController)
        msg=LoadController._request_class()
        msg.name='mobile_base_controller'
        loadController(msg)
        service='/rrbot/controller_manager/switch_controller'
        rospy.wait_for_service(service)
        switchController=rospy.ServiceProxy(service,SwitchController)
        msg=SwitchController._request_class()
        msg.start_controllers=['mobile_base_controller']
        #msg.STRICT=1 # best effort
        switchController(msg)
        print('Carga del controlador finalizada')

    def setAutomaticMode(self,tipo=8):
        if self.automaticMode!=0:
            self.automaticMode=0
            self.setPoint.linear.x=0
            self.setPoint.angular.z=0

        else:
            self.automaticMode=tipo
            tn=rospy.Time.now()
            now=tn.secs+tn.nsecs*1e-9-0.1
            self.t0=now
            self.phi0=0
            self.y0=0
            self.x0=0
            self.t=0

        print('Automatic mode setted in: ',self.automaticMode)

    def setPoint_0(self):
        t=self.t
        w_d=math.pi/30
        #radio=2.5
        phi=w_d*t
        vel=0.2
        radio=vel/w_d
        
        x=radio*math.sin(phi)
        y=-radio*(math.cos(phi)-1)# Para que empiece en (0,0)

        return x,y

    def setPoint_8(self):
        t=self.t
        w_max=0.2
        #dt=0.1
        w_d=w_max/6
        #radio=2.5
        phi=w_d*t
        vel_max=0.1 
        #vel=vel_max/2
        radio=math.sqrt(0.2)*vel_max/w_d
        #radio=vel_max

        x=radio*math.sin(2*phi)
        y=-radio*(math.cos(phi)-1)# Para que empiece en (0,0)

        return x,y
    
    def automaticSetPoint(self):
        """
        Mediante cinematica inversa se obtienen las senales de control
        diapo 50/218 de control de robots 2020
        """
        # Trayectoria
        """
        La trayectoria es un 8 x=cos(tita), y=sin(2*tita)
        """
        tn=rospy.Time.now()
        now=tn.secs+tn.nsecs*1e-9
        dt=now-self.t0
        self.t0=now
        self.t+=dt
        if self.automaticMode=='o':
            x,y=self.setPoint_0()
        elif self.automaticMode=='8':
            x,y=self.setPoint_8()

        #print('x,y= ',x,y) 
        """
        w_d=math.pi/10 # velocidad
        longitud=3

        longitud=longitud/2
        
        phi=w_d*self.t
        #print('phi: ',phi)
        
        #vx=-math.sin(phi)
        #vy=2*math.cos(2*phi)
        # trayectoria
        x=longitud*math.sin(phi)
        y=-longitud*math.cos(phi)
        """

        # Discrete derivative
        vx=(x-self.x0)/dt
        vy=(y-self.y0)/dt
        self.x0=x
        self.y0=y
        #print('vx: ',vx,'vy: ',vy)

        #vx=v_base*math.sin(phi)
        #vy=v_base*math.cos(2*phi)
        #print('vx: ',vx,'vy: ',vy)
        
        # Senales de control
        phi=math.atan2(vy,vx) # Ojo aca.
        
        dif=angular_difference(phi,self.phi0)
        w=dif/dt
        
        #print('phi: ',phi)
        self.phi0=phi
        #tita=remaind(phi,math.pi/2)
        tita=phi
        #print('phi: ',tita)
        
        u=vx*math.cos(tita)+vy*math.sin(tita)
        self.setPoint.linear.x=u # Ver esto...
        self.setPoint.angular.z=w
        print('linear: ',self.setPoint.linear.x,' angular: ', self.setPoint.angular.z)

def rotation(phi):
  #phi=np.pi*phi/180
  R=np.array([[np.cos(phi),-np.sin(phi)],
              [np.sin(phi),np.cos(phi)]])
  return R

def angular_difference(tita1,tita2):
  R1=rotation(tita1)
  R2=rotation(tita2)
  R3=np.matmul(R2,R1.T)
  ang=np.arctan2(R3[0,1],R3[0,0])#*180/np.pi
  return ang

def remaind(x,y):
    n=int(x/y)
    print('x ',x,'y ',y,'remaind: ',x-n*y )
    return x-n*y

if __name__ == '__main__':    
    try:
        pioneer=ControlAction(name='human_control')
        pioneer.run()
    except rospy.ROSInterruptException:
        pass
