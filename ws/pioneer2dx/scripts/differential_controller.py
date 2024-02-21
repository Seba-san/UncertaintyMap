#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
#from std_msgs.msg import String
import threading
import sys, select, termios, tty # Esto es para leer el teclado
import os
from gazebo_msgs.srv import SetModelState, DeleteModel
from gazebo_msgs.msg import ModelState
from std_srvs.srv import SetBool,SetBoolRequest
from controller_manager_msgs.srv import LoadController, SwitchController
import math
import numpy as np
from std_msgs.msg import Float64 as Float_command
import time


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

        msg= 'Los comandos disponibles son: '
        print(msg)
        for key, value in self.moveBindings.items():
            msg=key+' : '+value[2]
            print(msg)

        #self.beginController('/rrbot/joint_state_controller')
        #self.beginController('/rrbot/right_wheel_controller')
        #self.beginController('/rrbot/left_wheel_controller')
	
        #self.x = threading.Thread(target=self.controlAction)
        #self.controlAction()
        self.automaticMode=0 # Es para generar senales automaticas
        self.signo_cuadrado=1 # Sirve para girar una vuelta para un lado
        self.offset=0

    def getNameTopic(self):
        # Por ahora lo pongo asi. A futuro buscar una forma de automatizarlo
        self._name='/rrbot/mobile_base_controller/cmd_vel'
        self._left='/rrbot/left_wheel_controller/command'
        #self._left='/set_point_left'
        #self._left='/left_dc_motor/command'
        self._right='/rrbot/right_wheel_controller/command'
        #self._right='/set_point_right'
        #self._right='/right_dc_motor/command'
    
    def manual_velocity(self):
        """
        Convierte v y w en wl y wr. Ademas los publica.
        """
        w_max=1 # rad/s
        v_max=1 # m/s
        v=self.setPoint.linear.x
        w=self.setPoint.angular.z
        if abs(v)>v_max:
            v=v*v_max/abs(v)

        if abs(w)>w_max:
            w=w*w_max/abs(w)

        #B=0.34 # valores del model.xacro
        #B=0.4627 #0.49
        #B=0.575
        #B=0.626
        #B=0.617
        B=0.34
        #B=0.51 # valores luego de testear en simulador
        #r=0.11 # valores del model.xacro
        #r=0.10 # valores del model.xacro
        #r=0.097 # 
        #r=0.1328 # 
        #r=0.096 # 
        r=0.108 # 
        wl=Float_command()
        wr=Float_command()
        wr.data =v/r+B*w/(2*r) # Inversion de la matriz dada en diapo 13/218
        wl.data=v/r-B*w/(2*r)
        self.pub_wl.publish(wl)
        self.pub_wr.publish(wr)

    def run(self):
        #name=getNameTopic()
        self.pub = rospy.Publisher(self._name, Twist, queue_size=10)
        self.pub_wl=rospy.Publisher(self._left, Float_command, queue_size=10)
        self.pub_wr=rospy.Publisher(self._right, Float_command, queue_size=10)

        rospy.init_node(self.name, anonymous=True)
        self.time=0.1
        self.rate = rospy.Rate(1/self.time) # 10hz
        #setPoint=Twist()
        print('Nodo inicializado')
        while not rospy.is_shutdown() and self.on:
            #hello_str = "hello world %s" % rospy.get_time()
            #rospy.loginfo(hello_str)
            self.getCommand()

            # genera un setPoint automatico 
            if self.automaticMode!=0:
                self.automaticSetPoint()

            self.pub.publish(self.setPoint)# Sirve para groundtruth y para el
            self.setPoint.linear.y=0 # Cambiar esto...
            # controlador
            self.manual_velocity()
            self.rate.sleep()

        rospy.signal_shutdown('Nodo finalizado por el usuario')

    def _setPoint(self,acc=[0,0]):
        self.setPoint.linear.x+=acc[0]
        self.setPoint.angular.z+=acc[1]
        print('linear: ',self.setPoint.linear.x,' angular: ', self.setPoint.angular.z)
        if abs(self.setPoint.linear.x)<0.01:
            #print('set 0 lineal')
            self.setPoint.linear.x=0
        
        if abs(self.setPoint.angular.z)<0.01:
            self.setPoint.angular.z=0

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
            data=self.moveBindings.get(key)
            funcExe=data[0]
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
        'a':[self._setPoint,[0,step],'girar a la izquierda'],
        's':[self._setPoint,[-step,0],'ir hacia atras'],
        'd':[self._setPoint,[0,-step],'girar a la derecha'],
        'w':[self._setPoint,[step,0],'ir hacia delante'],
        'q':[self.killNode,0,'salir'],
        'x':[self.setIniciar,0,'aparecer al origen'],
        'r':[self.respawnModel,0,'respawnear'],
            '8':[self.setAutomaticMode,'8','Hacer un 8'],
            'o':[self.setAutomaticMode,'o','Hacer un circulo'],
            'c':[self.setAutomaticMode,'c','Hacer un cuadrado'],
            'l':[self.setAutomaticMode,'l','ir en linea recta'],
            'v':[self.setAutomaticMode,'v','girar 360 sobre el lugar']
           }
        self.on=True
   
    def setIniciar(self):
        """
        Comando para multiples robots
        """
        self.setPoint.linear.y=1
        print('Reseteando')




    def setIniciar_(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        #modelState=ModelState()
        setEstadoFun=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        modelState=ModelState()
        modelState.model_name='pioneer2dx'
        resp=setEstadoFun(modelState)
        print('set in xyz=[0,0,0], rpy=[0,0,0]')
        """
        llamar otro servicio o escuchar el servicio de gazebo, para reiniciar
        la odometria.
        """
        rospy.wait_for_service('/pioneer2dx/ground_truth/reset_odometry')
        reset_odometry=rospy.ServiceProxy('/pioneer2dx/ground_truth/reset_odometry',SetBool)
        command=SetBoolRequest()
        command.data=True
        resp=reset_odometry(command)
        print(resp)
    
    def respawnModel(self):
        print('Inicializando Respawn model')
        service='/gazebo/delete_model'
        rospy.wait_for_service(service)
        deleteModel=rospy.ServiceProxy(service,DeleteModel)
        #deleteModel(model_name='pioneer2dx')
        deleteModel(model_name='robot1')
        os.system("roslaunch pioneer2dx respawn.launch robot_name:=robot1")
        #os.system("roslaunch pioneer2dx multiple_robots.launch")
        #os.system("roslaunch pioneer2dx one_robot.launch robot_name:=robot1 init_pose:=-x 0 -y 0 -z 0.0")
        self.beginController_()
        print('Respawn Finalizado')

    def beginController_(self):
        print('configurando controlador')

        #self.beginController('/rrbot/right_wheel_controller')
        #self.beginController('/rrbot/left_wheel_controller')
        """
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
        """

    def beginController(self,controller):
        print('configurando controlador '+controller )
        service='/rrbot/controller_manager/load_controller'
        rospy.wait_for_service(service)
        loadController=rospy.ServiceProxy(service,LoadController)
        msg=LoadController._request_class()
        msg.name=controller
        loadController(msg)
        service='/rrbot/controller_manager/switch_controller'
        rospy.wait_for_service(service)
        switchController=rospy.ServiceProxy(service,SwitchController)
        msg=SwitchController._request_class()
        msg.start_controllers=[controller]
        #msg.STRICT=1 # best effort
        switchController(msg)
        print('Carga del controlador finalizada '+controller)

    def setAutomaticMode(self,tipo=8):
        if self.automaticMode!=0:
            self.automaticMode=0
            self.setPoint.linear.x=0
            self.setPoint.angular.z=0
            self.state=0

        else:
            self.automaticMode=tipo
            tn=rospy.Time.now()
            now=tn.secs+tn.nsecs*1e-9-0.1
            self.t0=now
            self.phi0=0
            self.y0=0
            self.x0=0
            self.t=0
            self.state=0

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
        w_max=0.1
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
        
        #if t>(1/w_d):
        #    x=0
        #    y=0
        return x,y

    def setPoint_Cuadrado(self):
        """
        con state=1 le da comandos directamente a las velocidades. x=vel,
        y=omega
        """
        sign=self.signo_cuadrado
        t=self.t
        period=20
        vel=0.2#+self.offset
        tita=math.pi/(2*period/2)
        self.state=1

        if t%period>period/2:
            x=0
            y=tita*sign
        else:
            x=vel
            y=0
        
        if t>period*4: #160 #20
            x=0
            y=0

        if t>period*4+5: #165
            self.offset+=1
            if self.offset>24:
                print('Se finalizaron las 25 vultas')
            else:
                print('Vuelta numero: ',self.offset)
                self.setIniciar() # Reinicia la vuelta y la odometria.
                tn=rospy.Time.now()
                now=tn.secs+tn.nsecs*1e-9-0.1
                self.t0=now
                self.phi0=0
                self.y0=0
                self.x0=0
                self.t=0
                self.state=0
            #self.signo_cuadrado=-self.signo_cuadrado
           # if self.signo_cuadrado>0:
           #     self.offset+=0.1
            #self.setAutomaticMode(tipo='c')
        return x,y

    def setPoint_linea_recta(self):
        """
        toma la posicion actual y mantiene la direccion??.
        """
        t=self.t
        vel_max=0.5
        w_max=0.1
        x=0
        y=0

        return x,y

    def setPoint_vuelta(self):
        """
        Hace una vuelta de 360, espera 5 seg y reinicia
        """
        t=self.t
        self.state=1
        w=0.1
        vel=0

        if t>math.pi*2/w+5: 
            self.setIniciar() # Reinicia la vuelta y la odometria.
            tn=rospy.Time.now()
            now=tn.secs+tn.nsecs*1e-9-0.1
            self.t0=now
            self.phi0=0
            self.y0=0
            self.x0=0
            self.t=0
            self.state=0
        
        if t>math.pi*2/w:
            w=0
        
        return vel,w

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
        elif self.automaticMode=='l':
            x,y=self.setPoint_linea_recta()
        elif self.automaticMode=='c':
            x,y=self.setPoint_Cuadrado()
        elif self.automaticMode=='v':
            x,y=self.setPoint_vuelta()

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

        if self.state!=0:
            self.setPoint.linear.x=x # Ver esto...
            self.setPoint.angular.z=y
            self.state=0

        msg='linear: '+str(self.setPoint.linear.x)+' angular: '
        msg=msg+str(self.setPoint.angular.z)+ ' tita: '+str(tita)
        msg=msg+'         '+str(time.time())+'                   '

        sys.stdout.write("\033[F")
        print(msg)
        #sys.stdout.write("\r"+msg)

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
