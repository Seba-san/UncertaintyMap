#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates, ModelState, LinkState  
from gazebo_msgs.srv import GetModelState, SetModelState, JointRequest, GetLinkState
from gazebo_msgs.srv import BodyRequest, GetModelProperties
from gazebo_msgs.srv import ApplyJointEffort, ApplyBodyWrench 
from gazebo_msgs.srv import ApplyJointEffortRequest,SetLinkState
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
import copy
import time
#import numpy as np
import os
#Esto es para comandar desde el teclado
import sys, select, termios, tty 
#from std_msgs.msg import String

def factoryModelStateBegins(modelState=ModelState(),pos=[0,0,0],ori=[0,0,0,0],vl=[0,0,0],va=[0,0,0]):
    modelState.pose.position.x=pos[0]
    modelState.pose.position.y=pos[1]
    modelState.pose.position.z=pos[2]
    modelState.pose.orientation.x=ori[0]
    modelState.pose.orientation.y=ori[1]
    modelState.pose.orientation.z=ori[2]
    modelState.pose.orientation.w=ori[3]
    modelState.twist.linear.x=vl[0]
    modelState.twist.linear.y=vl[1]
    modelState.twist.linear.z=vl[2]
    modelState.twist.angular.x=va[0]
    modelState.twist.angular.y=va[1]
    modelState.twist.angular.z=va[2]
    return modelState
    
def factoryModelState(modelState=ModelState()):
    modelState=getData()

def factoryCopyState(modelState=ModelState()):
    state=getData()
    modelState.pose=copy.deepcopy(state.pose)
    modelState.twist=copy.deepcopy(state.twist)
    return modelState

def inspector(data):
    Info=data.name
    for link in Info:
        if link=='pioneer2dx::base_link':
            i=Info.index(link)
            break

    #Info=data.deserialize
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.twist[i])
    #LinkStates.

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/gazebo/link_states',LinkStates,inspector)
    rospy.spin()
    
def getData():
    rospy.wait_for_service('/gazebo/get_model_state')
    getEstado=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    varEstado=getEstado(model_name='pioneer2dx')
    #print "Service of pioneer2dx :\n", varEstado.twist
    return varEstado

    #rospy.loginfo("Service %s",varEstado.twist)

def setIniciar():
    rospy.wait_for_service('/gazebo/set_model_state')
    #modelState=ModelState()
    setEstadoFun=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
    modelState=factoryModelStateBegins()
    modelState.model_name='pioneer2dx'
    resp=setEstadoFun(modelState)
    #print resp

def setRunForward():
    rospy.wait_for_service('/gazebo/set_model_state')
    setEstadoFun=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
    modelState=factoryCopyState()
    #factoryModelState(vl=[1,0,0])
    modelState.model_name='pioneer2dx'
    modelState.twist.linear.x=1
    modelState.twist.linear.y=0 
    modelState.twist.linear.z=0
    modelState.reference_frame='pioneer2dx'
    #modelState.pose.orientation.x=0  
    #modelState.pose.orientation.y=0
    #modelState.pose.orientation.z=0
    #modelState.pose.orientation.w=0
    resp=setEstadoFun(modelState)
    #print resp

def runForward():
    setIniciar()
    while True:
        modelState=getData()
        velocity=modelState.twist.linear 
        if velocity.x<1:
            setRunForward()

        time.sleep(0.1)


def forces():
    service='/gazebo/apply_joint_effort'
    rospy.wait_for_service(service)
    setJointEffort=rospy.ServiceProxy(service,ApplyJointEffort)
    effort=ApplyJointEffortRequest()
    effort.joint_name='left_wheel_hinge'
    effort.effort=2
    effort.duration.secs=-1
    resp=setJointEffort(effort)
    print resp

def clearAllEfforts():
    # Esta funcion borra todos los esfuerzos existentes sobre un objeto
    # Lectura de todos los link y joints
    service='/gazebo/get_model_properties'
    rospy.wait_for_service(service)
    getModelProperties=rospy.ServiceProxy(service,GetModelProperties)
    resp=getModelProperties(model_name='pioneer2dx')  
    #resp=GetModelProperties._response_class.serialize
    #mostrar=resp.serialize
    #print type(resp.joint_names )
    for joint in resp.joint_names:
        service='/gazebo/clear_joint_forces'
        rospy.wait_for_service(service)
        clearJointEffort=rospy.ServiceProxy(service,JointRequest)
        resp1=clearJointEffort(joint)
        #print resp1
    
    for body in resp.body_names:
        service='/gazebo/clear_body_wrenches'
        rospy.wait_for_service(service)
        clearBodyEffort=rospy.ServiceProxy(service,BodyRequest)
        resp1=clearBodyEffort(body)
        #print resp1

def applyForce(wheel='left',effort=0):
    service='/gazebo/apply_joint_effort'
    rospy.wait_for_service(service)
    setJointEffort=rospy.ServiceProxy(service,ApplyJointEffort)
    effortMsg=ApplyJointEffortRequest()
    effortMsg.joint_name=wheel+'_wheel_hinge'
    effortMsg.effort=effort
    effortMsg.duration.secs=3
    resp=setJointEffort(effortMsg)
    #print resp


def applyControl(u):
    if u>0.5:
        u=0.5
    elif u<-0.5:
        u=-0.5
    
    effortBias=1
    effortLeft=u/2
    effortRight=-u/2
    applyForce('left',effortLeft)
    applyForce('right',effortRight)

def controlador():
    u=0
    effortBias=1
    applyForce('left',effortBias)
    applyForce('right',effortBias)
    while True:
        estado=getData()
        vel=[0,0,0]
        vel[0]=estado.twist.linear.x
        vel[1]=estado.twist.linear.y
        vel[2]=estado.twist.linear.z
        print "velocidad "+ str(max(vel))
        if max(vel)>0.5:
            print ("MAX VEL!!!!!!!!!!!!!!!!!!!")
            clearAllEfforts()
            u=0;
            time.sleep(1)
            applyForce('left',effortBias)
            applyForce('right',effortBias)


        sensado=estado.pose.position.y
        print("pos en y: ", sensado)
        kp=100
        ki=-1
        kd=0.1
        uAnt=u
        #clearAllEfforts()
        u=-kp*sensado+u*ki
        u=u+kd*(u-uAnt)
        print "signal control "+ str(u)
        applyControl(u)
        time.sleep(0.2)

def getKey():
# Fuente principal: https://github.com/turtlebot/turtlebot/blob/melodic/turtlebot_teleop/scripts/turtlebot_teleop_key
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def salir():
    return -1

def getLinkState(link_name_='',reference=''):
    service='/gazebo/get_link_state'
    rospy.wait_for_service(service)
    getLinkState=rospy.ServiceProxy(service,GetLinkState)
    return getLinkState(link_name=link_name_,reference_frame=reference)
 
def setVelocityWheels():
    leftWheelIni=getLinkState('left_wheel','base_link')
    rightWheelIni=getLinkState('right_wheel','base_link')
    leftWheelFin=copy.deepcopy(leftWheelIni.link_state)
    rightWheelFin=copy.deepcopy(rightWheelIni.link_state)
    leftWheelFin.reference_frame='base_link'
    rightWheelFin.reference_frame='base_link'

    leftWheelFin.twist.angular.y=10
    rightWheelFin.twist.angular.y=10
    service='/gazebo/set_link_state'
    rospy.wait_for_service(service)
    setLinkState=rospy.ServiceProxy(service,SetLinkState)
    setLinkState(leftWheelFin)
    setLinkState(rightWheelFin)

def setForceWheels(direction=[1,1]):
            applyForce('left',direction[0])
            applyForce('right',direction[1])

def setVelocity(linear=0,angular=0):
    if abs(linear)>10:
        angular=copy.deepcopy(linear)
        angular=angular-10
        linear=0
       
    #a=GetLinkState._response_class()
    #a.
    linkStateIni=getLinkState('base_link')
    linkStateFin=LinkState()
    linkStateFin=copy.deepcopy(linkStateIni.link_state)
    #linkStateFin.twist=copy.deepcopy(linkStateIni.twist)
    linkStateFin.link_name='base_link'
    #print(linkState)
    service='/gazebo/set_link_state'
    rospy.wait_for_service(service)
    setLinkState=rospy.ServiceProxy(service,SetLinkState)
    
    #modelState=ModelState()
    #factoryModelState(vl=[1,0,0])
    #linkState=LinkState()
    linkStateFin.reference_frame='base_link'
    if linear!=0:
        linkStateFin.twist.linear.x=linear
    else:
        linkStateFin.twist.angular.z=angular
    
    resp=setLinkState(linkStateFin)


def respawnModel():
    service='/gazebo/delete_model'
    rospy.wait_for_service(service)
    deleteModel=rospy.ServiceProxy(service,DeleteModel)
    deleteModel(model_name='pioneer2dx')
    os.system("roslaunch pioneer2dx respawn.launch")
    """
    service='/gazebo/spawn_urdf_model'
    rospy.wait_for_service(service)
    spawnModel=rospy.ServiceProxy(service,SpawnModel)
    obj=SpawnModel._request_class()
    obj.model_name='pioneer2dx'
    obj.model_xml='../models/model'
    spawnModel(obj)
    """

def keyBoardOperation():
    moveBindings = {
        'a':[setForceWheels,[0.2,-0.2]],
        's':[setForceWheels,[-0.2,-0.2]],
        'd':[setForceWheels,[-0.2,0.2]],
        'w':[setForceWheels,[0.2,0.2]],
        'q':[salir,0],
        'x':[setIniciar,0],
        'r':[respawnModel,0]
           }

    on=True
    while on:
        key=getKey()
        #print("tecla precionada: "+key+"\n")
        if key in moveBindings.keys():
            funcExe,_=moveBindings.get(key)
            if moveBindings[key][1]!=0:
                out=funcExe(moveBindings[key][1])
            else:
                out=funcExe()

            if out==-1:
               on=False
        time.sleep(0.1)
        
def print_instrucciones():
    msg= "Controlador inicializado... "
    print(msg)
    msg= "con las teclas [a][s][d][w] se movera a una velocidad de 0.2, por ejemplo con [a] el comando de movimiento es [0.2, -0.2] "
    print(msg)
    msg= "Con [q] se sale, [x] vuelve a colocarlo en el origen y [r] borra el robot y respawnea.  "
    print(msg)


if __name__== '__main__':
    # listener()
    #runForward()
    setIniciar()
    clearAllEfforts()
    for i in range(3):
        time.sleep(1)
        print(i)
    #clearAllEfforts()
    #time.sleep(3)
    #forces()
    #controlador()
    #runForward()
    print_instrucciones()
    keyBoardOperation()
