#!/usr/bin/env python
# Este nodo toma los datos de la pose del robot y las publica en un topic
# separado.
import numpy as np
import rospy
#from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import SetModelState, DeleteModel
from gazebo_msgs.msg import LinkStates, ModelState, LinkState  
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped,Twist
from std_srvs.srv import SetBool,SetBoolResponse 
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,Float64 
import tf2_ros
#import tf.transformations as Transformations
import copy
import tf
import math
import sys
#import os


class Remapper():
    """
    Remapea la posicion y la orientacion del pioneer, con el objetivo de usarlo
    como grountruth. Hace las veces de puente entre gazebo y el pioneer.
    Revisar xq la pose publicada, no coincide con la TF
    """
    def __init__(self,pose_ini):
        self.name_space=rospy.get_namespace()
        self.get_pose_ini(pose_ini)
        self.index=-1
        self.reseted=True
        self.decimated=0
        self.control=Twist()
        self.init_odometry()
        # Set covariance matrix
        s=0.001
        sw=100
        self.odo.pose.covariance=[0.]*36
        self.odo.twist.covariance=[0.]*36
        for i in range(3):
            self.odo.pose.covariance[i*6]=s
            self.odo.twist.covariance[i*6]=s
            self.odo.pose.covariance[i*6+18]=sw
            self.odo.twist.covariance[i*6+18]=sw
        
        self.init_pose()
        self.init_tf()
        self.init_repeater()
        self.init_node()
        self.init_reset_service()
        self.listener()
    def get_pose_ini(self,pose_ini):
        """
        Determina la pose inicial para luego setearla. El formato es el
        siguiente: "-x 1 -y 1 -z 0"
        """
        #dato = pose_ini.rsplit("-")
        dato = pose_ini
        rospy.logwarn('probando 123 {}'.format(pose_ini))
        self.pose_ini={}
        self.pose_ini['x']=float(dato[1])
        self.pose_ini['y']=float(dato[3])
        self.pose_ini['z']=float(dato[5])


    def init_repeater(self):
        """
        Remapea los topics desde el command central a cada topic individual con
        su propio nameSpace
        """
        self.left_repeater=rospy.Publisher(self.name_space+'left_wheel_controller/command',Float64,queue_size=1)
        self.right_repeater=rospy.Publisher(self.name_space+'right_wheel_controller/command',Float64,queue_size=1)
        self.cmd_vel_repeater=rospy.Publisher(self.name_space+'mobile_base_controller/cmd_vel',Twist,queue_size=1)

    def init_reset_service(self):
        self.reset_odom = rospy.Service(self.name_space+'pioneer2dx/ground_truth/reset_odometry', SetBool,self.reset_odometry)
    
    def init_pose(self):
        self.poseStamped=PoseStamped()
        self.poseStamped.header.frame_id="world"
        self.poseStamped.header.seq=0
        self.pub_pose =rospy.Publisher(self.name_space+'pioneer2dx/ground_truth/pose',PoseStamped,queue_size=1)

    def init_tf(self):
        """
        publica la transformacion groundTruth entre el mundo y la base_link
        """
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        # para que sea compatabile hay que agregarle _tf:
        ns=self.name_space
        ns=ns[0:-1]+'_tf/'

        #self.t.header.frame_id=self.name_space+"odom_groundtruth"
        self.t.header.frame_id=ns+"odom_groundtruth"

        #self.t.child_frame_id="base_link_ground_truth"
        #self.t.child_frame_id=self.name_space+"base_link"
        self.t.child_frame_id=ns+"base_link"
        #self.t.header.frame_id="base_link"
        #self.t.child_frame_id="odom_groundtruth"

    def init_odometry(self):
        """
        Configuracion de la odometria ground Truth
        """
        self.odo=Odometry()
        self.odo.pose.pose.orientation.w=1
        self.odo.pose.pose.orientation.x=0
        self.odo.pose.pose.orientation.y=0
        self.odo.pose.pose.orientation.z=0
        self.odo.pose.pose.position.x=self.pose_ini['x']
        self.odo.pose.pose.position.y=self.pose_ini['y']
        self.odo.pose.pose.position.z=self.pose_ini['z']

        self.odo.header.frame_id='odom_groundtruth'
        self.odo.child_frame_id='base_link'
        #self.odo.header.frame_id='base_link'
        #self.odo.child_frame_id='odom_groundtruth'
        self.pub_odo =rospy.Publisher(self.name_space+'pioneer2dx/ground_truth/odom',Odometry,queue_size=1)
       
    def reset_odometry(self,data):
        self.odo.pose.pose.orientation.w=1
        self.odo.pose.pose.orientation.x=0
        self.odo.pose.pose.orientation.y=0
        self.odo.pose.pose.orientation.z=0
        self.odo.pose.pose.position.x=self.pose_ini['x']
        self.odo.pose.pose.position.y=self.pose_ini['y']
        self.odo.pose.pose.position.z=self.pose_ini['z']
        rta=SetBoolResponse()
        rta.success=True
        rta.message='Odometria Reiniciada'
        return rta

    def reset_pose(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        setEstadoFun=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        modelState=ModelState()
        modelState.model_name=self.name_space[1:-1]
        
        modelState.pose.position.x=self.pose_ini['x']
        modelState.pose.position.y=self.pose_ini['y']
        modelState.pose.position.z=self.pose_ini['z']        
        resp=setEstadoFun(modelState)
        """
        print('set in xyz=[{x},{y},{y}],\
                rpy=[0,0,0]'.format(x=self.pose_ini['x'],y=self.pose_ini['y'],z=self.pose_ini['z']))
        """
        # Reseteo odometria
        self.odo.pose.pose.orientation.w=1
        self.odo.pose.pose.orientation.x=0
        self.odo.pose.pose.orientation.y=0
        self.odo.pose.pose.orientation.z=0
        self.odo.pose.pose.position.x=self.pose_ini['x']
        self.odo.pose.pose.position.y=self.pose_ini['y']
        self.odo.pose.pose.position.z=self.pose_ini['z']
        rospy.loginfo('reseteando {}'.format(self.name_space[1:-1]))

    def run_tf(self,pos=PoseStamped()):
        #self.t.transform.translation.deserialize(pos.pose.position.serialize)
        #self.t.transform.rotation.deserialize(pos.pose.orientation.serialize)
        
        p=copy.deepcopy(pos)
        self.t.transform.translation.x=p.pose.position.x
        self.t.transform.translation.y=p.pose.position.y
        self.t.transform.translation.z=p.pose.position.z

        q=[]
        q.append(p.pose.orientation.z)
        q.append(p.pose.orientation.y)
        q.append(p.pose.orientation.x)
        q.append(p.pose.orientation.w)
        #print('pose ',q)

        #q2=Transformations.quaternion_inverse(q)
        #print('inversa', q2)
        #print('inversa 1', q2[0])
        #print('inversa 2 ', q2[1])
        #print('inversa 3', q2[2])
        self.t.transform.rotation.w=q[3]
        self.t.transform.rotation.x=q[2]
        self.t.transform.rotation.y=q[1]
        self.t.transform.rotation.z=q[0]

        self.t.header.stamp=pos.header.stamp
        self.t.header.seq=pos.header.seq
    
    def run_tf(self,pos=Odometry()):
        #self.t.transform.translation.deserialize(pos.pose.position.serialize)
        #self.t.transform.rotation.deserialize(pos.pose.orientation.serialize)
        
        #p=copy.deepcopy(pos.pose)
        p=copy.deepcopy(pos)
        self.t.transform.translation.x=p.pose.position.x
        self.t.transform.translation.y=p.pose.position.y
        self.t.transform.translation.z=p.pose.position.z

        q=[]
        q.append(p.pose.orientation.z)
        q.append(p.pose.orientation.y)
        q.append(p.pose.orientation.x)
        q.append(p.pose.orientation.w)
        #print('pose ',q)

        #q2=Transformations.quaternion_inverse(q)
        #print('inversa', q2)
        #print('inversa 1', q2[0])
        #print('inversa 2 ', q2[1])
        #print('inversa 3', q2[2])
        self.t.transform.rotation.w=q[3]
        self.t.transform.rotation.x=q[2]
        self.t.transform.rotation.y=q[1]
        self.t.transform.rotation.z=q[0]

        self.t.header.stamp=pos.header.stamp
        self.t.header.seq=pos.header.seq
   
    def euler_from_quaternion(self, R):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        Se toma q=[w,x,y,z]
        """
        #x=R[0]
        #y=R[1]
        #z=R[2]
        #w=R[3]
        
        w=R[0]
        x=R[1]
        y=R[2]
        z=R[3]
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def euler_to_quaternion(self,r):
        (roll, pitch, yaw) = (r[0], r[1], r[2])
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qw, qx, qy, qz]

    def get_state_estimate(self):
        """
        Simula la odometria solo usando el modelo para estimar su estado.
        """

        #get dt
        tn=rospy.Time.now()
        now=tn.secs+tn.nsecs*1e-9
        t=self.odo.header.stamp
        before=t.secs+t.nsecs*1e-9
        dt=now-before
        self.odo.header.stamp=copy.deepcopy(tn)
        # get pose before
        pos=self.odo.pose.pose.position
        q=self.odo.pose.pose.orientation
        #rpy=tf.transformations.euler_from_quaternion([q.w,q.x,q.y,q.z])
        rpy=self.euler_from_quaternion([q.w,q.x,q.y,q.z])
        print('rpy: ',rpy)
        tita=rpy[2]
        X0=np.array([[pos.x,pos.y,tita]]).T
        
        # get control signal
        v=self.control.linear.x
        # No se xq el controlador lo setea a la
        #mitad del deseado
        z=self.control.angular.z 
        # calc new pose
        if z==0:
            non_linear=np.array([[v*dt*np.cos(tita),
                                  v*dt*np.sin(tita),
                                  0]]).T
        else:
            r=v/z
            # ver cheng1997
            non_linear=np.array([[r *(- np.sin(tita)+np.sin(tita+z*dt)),
                                  r *(+np.cos(tita)-np.cos(tita+z*dt)) ,
                                  z*dt]]).T
        
        X1=X0+non_linear
        self.odo.pose.pose.position.x=X1[0][0]
        self.odo.pose.pose.position.y=X1[1][0]
        
        #q=tf.transformations.quaternion_from_euler(rpy[0],rpy[1],X1[2][0])
        q=self.euler_to_quaternion([rpy[0],rpy[1],X1[2][0]])
        
        #print('ang: ',X1[2][0])
        rpy=self.euler_from_quaternion(q)
        #print('rpy2: ',rpy)
        """
        self.odo.pose.pose.orientation.z=q[2]
        self.odo.pose.pose.orientation.y=q[1]
        self.odo.pose.pose.orientation.x=q[0]
        self.odo.pose.pose.orientation.w=q[3]
        """
        #"""
        self.odo.pose.pose.orientation.z=q[3]
        self.odo.pose.pose.orientation.y=q[2]
        self.odo.pose.pose.orientation.x=q[1]
        self.odo.pose.pose.orientation.w=q[0]
        #"""
        self.odo.twist.twist.linear.x=v
        self.odo.twist.twist.angular.z=z
        #X1[2][0]=q[2]
        #print('pose: ',X1.T,'quaternion: ',q,'dt: ',dt)
        print('control v: ',v,'angular z: ',z, 'time t:',dt)
        #rospy.loginfo('pose {}'.format(dt))

    def run_odometry(self,data):
        # son 2 funciones, depende de cual sea el argumento de entrada
        self.get_state_estimate()
        self.odo.header.seq=copy.deepcopy(self.poseStamped.header.seq)
        # Aca hay que meter la magia para simular la odometria
        #self.odo.pose.covariance=[0.]*36 # Ver esto despues
        #self.odo.twist.twist=data.twist[self.index]
        #self.odo.twist.covariance=[0.]*36 # Ver esto despues
        self.pub_odo.publish(self.odo)

    def run_odometry(self):
        self.get_state_estimate()
        self.odo.header.seq+=1
        # Aca hay que meter la magia para simular la odometria
        #self.odo.pose.covariance=[0.]*36 # Ver esto despues
        #self.odo.twist.twist=data.twist[self.index]
        #self.odo.twist.covariance=[0.]*36 # Ver esto despues
        self.pub_odo.publish(self.odo)
        
        # Modo experimental $3
        # 10hz
        #self.get_state_estimate()
        self.run_tf(self.odo)
        self.br.sendTransform(self.t)



    def run_pose(self, data):
        self.poseStamped.pose=data.pose[self.index]
        self.poseStamped.header.stamp=rospy.Time.now()
        self.poseStamped.header.seq +=1
        self.pub_pose.publish(self.poseStamped)

    def inspector(self, data):
        Info=data.name
        if Info[self.index]!=self.name_space[1:-1]+'::base_link':
            for link in Info:
                if link==self.name_space[1:-1]+'::base_link':
                    i=Info.index(link)
                    self.index=i
                    break

        if self.decimated>50:
            # Publica la pose
            self.run_pose(data)
            self.decimated=0
        

            # Publica la transformacion entre la pose y el mundo.
            self.run_tf(self.poseStamped) # no funciona, 'Pose' object has no attribute 'pose'
            #self.run_tf(self.odo)
            self.br.sendTransform(self.t) # no funciona, el quaternion esta mal armado.
            # Publica la odometria
            #self.run_odometry(data)
        else:
            self.decimated+=1
        
    def state_estimate(self,data):
        # Fs=10Hz
        self.control=copy.deepcopy(data)
        self.run_odometry()

    def right_command(self,data):
        self.right_repeater.publish(data)

    def left_command(self,data):
        self.left_repeater.publish(data)

    def cmd_vel_command(self,data):
        if int(data.linear.y)==1 and self.reseted:
            self.reset_pose()
            self.reseted=False
        else:
            self.reseted=True

        self.cmd_vel_repeater.publish(data)

    def listener(self):
        #rospy.init_node('remapper_pioneer2dx_stat', anonymous=True)
        rospy.Subscriber('/gazebo/link_states',LinkStates,self.inspector)
        #rospy.Subscriber(self.name_space+'mobile_base_controller/cmd_vel',Twist,self.state_estimate)
        rospy.Subscriber('/rrbot/mobile_base_controller/cmd_vel',Twist,self.state_estimate)
        #self.r=rospy.Rate(1000)
        # Replica los comandos
        rospy.Subscriber('/rrbot/left_wheel_controller/command',Float64,self.left_command)
        rospy.Subscriber('/rrbot/right_wheel_controller/command',Float64,self.right_command)
        rospy.Subscriber('/rrbot/mobile_base_controller/cmd_vel',Twist,self.cmd_vel_command)
        rospy.spin()

    def init_node(self):
        #os.system("rosnode kill /robot1*")
        rospy.init_node('remapper_pioneer2dx_state',anonymous=True)
        self.r=rospy.Rate(1000)

    def run(self):
        while not rospy.is_shutdown():
           #self.pub.publish(self.pose)
           self.r.sleep()


def main(pose_ini):
    #print(LinkStates())
    #listener()
    remap=Remapper(pose_ini)
    remap.run()
    #Remapper.run()
    return


if __name__=='__main__':
    pose_ini=sys.argv[1:]
    main(pose_ini)
