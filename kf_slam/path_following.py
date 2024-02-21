#!/usr/bin/env python3
"""catch the data from slam and map, do a simulation and send control signals to the robot to do a path following
    
To do this, this node call a service when is needed to do the simulation. After the simulation 
is done, the node send the control signals to the robot.

The control signals needs to do some movements, first we need do this:
.- get the real orientation of the robot
.- calculate the orientation needed to do the movement
.- apply a control law to set the correct orientation
.- when the orientation is correct (+- 5 degrees), forward control was applied to do the movement
.- The error of forward control is the distance to the next waypoint and use this to determine final conditoin (less 1.0m)

The previous steps are like a path following, with waypoints. When the previous steps are done,
the loop begin again.
"""
    
    
import rospy
import roslaunch
from gazebo_msgs.msg import ModelStates as models
import numpy as np
from pid import PID
from std_msgs.msg import Float32 as Control_command
from std_msgs.msg import Float64 
import tf2_ros
from geometry_msgs.msg import Point as dot_command
import tf
import time

class Position_controller:
    """_summary_ This class is used to controlate the position of the robot using the SLAM2D data source
    
    """
    def __init__(self):
        #self.tfBuffer = tf2_ros.Buffer()   
        self.PID=PID()
        self.name_space='/robot1'
        self.node_name=self.name_space+'/position_controller/'
        D={'kp':0.1,'ki':0.01, 'kd':1.0, 'Ts':0.1}
        self.PID.set_parameters(Kp=D['kp'] , Ki=D['ki'], Kd=D['kd'],Ts=D['Ts'])
        if not rospy.has_param(self.node_name+'gains'):
            rospy.set_param(self.node_name+'gains', {'kp': D['kp'], 'ki':D['ki'], 'kd': D['kd']})
        
        self.set_point=[0,0]
        self.pose=[0,0]
        self.err=0
        self.sign=1
                                   
    def run(self):
        try:
            (trans,rot) = self.tfBuffer.lookup_transform('/robot1_tf/odom_groundtruth', '/kf_slam', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            
        x=trans[0]
        y=trans[0]
        self.pose=[x,y]
        self.set_point=[0,0]
    
    def compute_u(self):
        # Compute and send command signal 
        kp=rospy.get_param(self.node_name+'gains/kp')
        ki=rospy.get_param(self.node_name+'gains/ki')
        kd=rospy.get_param(self.node_name+'gains/kd')
        self.PID.set_parameters(Kp=kp , Ki=ki, Kd=kd,Ts=0.1)
        err=np.sqrt((self.set_point[0]-self.pose[0])**2+(self.set_point[1]-self.pose[1])**2)  
        err=err*self.sign
        self.command=self.PID.run(y_sp_=0,y_=-err)
        self.error=err
        
    def send_control_signal(self):

        v=self.command
        self.vr=v
        self.vl=v
        #self.wheels_velocity_l.publish(vl)
        #self.wheels_velocity_r.publish(vr)
        #self.vr=vr
    
    def get_setpoint(self,data=dot_command()):
        
        self.set_point=[data.x,data.y]
    

class Orientation_controller:
    """this class is used to controlate the orientation of the robot
    Saca muchas cosas de SLAM2D class.
    TODO: quizas SLAM2D podria sacar cosas de esta clase.
    """
    def __init__(self):
        #self.name_space=rospy.get_namespace()
        #self.name_space=self.name_space[0:-1] # WARNING: if launch from terminal, topic_left_wheel and right_wheel are not correct
        self.name_space='/robot1'
        topic_robot_state='/gazebo/model_states'
        self.orientation_sub=rospy.Subscriber(topic_robot_state,models,self.get_orientation,queue_size=1)        
        topic_left_wheel=self.name_space+'/left_wheel_controller/command'
        self.wheels_velocity_l=rospy.Publisher(topic_left_wheel,Float64,queue_size=1)
        topic_right_wheel=self.name_space+'/right_wheel_controller/command'
        self.wheels_velocity_r=rospy.Publisher(topic_right_wheel,Float64,queue_size=1)
        
        set_point_topic_name=self.name_space+'/orientation_controller/set_point'
        self.set_point_topic=rospy.Subscriber(set_point_topic_name,Control_command,self.get_setpoint,queue_size=1)
        
        self.PID=PID()
        self.node_name=self.name_space+'/orientation_controller/'
        D={'kp':0.5,'ki':0.0, 'kd':0.0, 'Ts':0.1}
        self.PID.set_parameters(Kp=D['kp'] , Ki=D['ki'], Kd=D['kd'],Ts=D['Ts'])
        if not rospy.has_param(self.node_name+'gains'):
            rospy.set_param(self.node_name+'gains', {'kp': D['kp'], 'ki':D['ki'], 'kd': D['kd']})
        
        self.PID.limit=2.0
        self.set_point=0
        self.base_line=0.34#m
        self.wheel_radius=0.108 #m
        self.block_callback=False        
    
    def get_setpoint(self,data=Control_command()):
        self.set_point=data.data
                   
    def get_orientation(self,data=models()):
        """
        get orientation from gazebo and calculate rotation
        Input:
            .- gazebo model
        Output:
            .- self.orientation: Orientation of robot in radians
        """
        if self.block_callback: # avoid callback
            return
        n_=len(data.name)
        robot_name='robot1'        
        if robot_name in data.name:
            for i in range(n_):
                if data.name[i]==robot_name:
                    break
        else:
            rospy.logerr('No se encontro el robot ' +robot_name+' dentro de los modelos de gazebo')        
            return
        qz=data.pose[i].orientation.z
        qw=data.pose[i].orientation.w
        self.calculate_orientation([qz,qw])
            
    def calculate_orientation(self,data):
        qz=data[0]
        qw=data[1]
        self.orientation=np.math.atan2(2*(qz*qw),1-2*(qz**2))
            
    def compute_u(self):
        # Compute and send command signal 
        
        kp=rospy.get_param(self.node_name+'gains/kp')
        ki=rospy.get_param(self.node_name+'gains/ki')
        kd=rospy.get_param(self.node_name+'gains/kd')
        self.PID.set_parameters(Kp=kp , Ki=ki, Kd=kd,Ts=0.1)
        x_sp=np.math.cos(self.set_point)
        y_sp=np.math.sin(self.set_point)
        x_me=np.math.cos(self.orientation)
        y_me=np.math.sin(self.orientation)
        err=np.math.acos(x_sp*x_me+y_sp*y_me) -np.pi/2 # -pi/2 is for avoid oscilations  
        dis=np.math.sqrt((x_sp-x_me)**2+(y_sp-y_me)**2)
        if dis> np.math.sqrt(2):# if the distance between setpoint and orientation is greater than 90 degrees set the error to pi/2
            err=np.pi/2 *np.sign(err)
                 
        self.command=self.PID.run(y_sp_=0,y_=-err)
    
    def send_control_signal(self):
        # get self.command and send setpoints to left and right wheels
        # One lap by second is 2*pi rad/s
        w=self.command*2*np.pi
        l=self.base_line
        vr=w*l/2
        vl=-w*l/2
        self.wheels_velocity_l.publish(vl)
        self.wheels_velocity_r.publish(vr)
        self.vr=vr
           
    def run(self):
        rospy.init_node('orientation_controller',anonymous=True)
        self.time=0.1
        self.rate = rospy.Rate(1/self.time) #hz
        
        while not rospy.is_shutdown():
            self.block_callback=True
            # main code is here
            self.compute_u()
            self.send_control_signal()
            #print('setpoint: '+str(self.set_point),'orientation: '+str(self.orientation),'comando: '+str(self.command),'velocidad r: '+str(self.vr))            
            # end main code
            self.block_callback=False
            self.rate.sleep()
            
class Master_Controller:
    """ This class implements orientation and position controller to implement a path following algorithm
    Input:
        .- Topics subscribed: 
            .- /dot_command: relative pose objetive
            .- /gazebo/model_states: absolute orientation 
            .- /robot1/left_wheel_controller/command : set point for left wheel PID controller
            .- /robot1/right_wheel_controller/command : set point for right wheel PID controller
            .- Transforms from /robot1_tf/odom_groundtruth to /kf_slam 
        .- Parameters:
            .- /robot1/orientation_controller/gains/kp, ki, kd: PID gains for orientation controller
            .- /robot1/position_controller/gains/kp, ki, kd: PID gains for position controller
                
    The Main implementation is in Orientation_controller class and run() method.
    """
    def __init__(self):
        dot_command_topic_name='/dot_command'
        self.dot_command_topic=rospy.Subscriber(dot_command_topic_name,dot_command,self.get_setpoints,queue_size=1)
        self.debug=rospy.Publisher('/debug',Float64,queue_size=1)
        self.OC=Orientation_controller()
        self.PC=Position_controller()

        self.ori_setpoint=0
        self.forward_setpoint=[0,0]
        self.block_callback=False
        self.set_point=[0,0]
        self.histereis=True
                
    def get_setpoints(self,data=dot_command()):
        """ this method is used to get the setpoint from the dot_command topic
        Input:
            .- data: dot_command message
        Output:
            .- self.ori_setpoint: orientation setpoint
            .- self.forward_setpoint: position setpoint            
        """
        x_obj=data.x
        y_obj=data.y   
    
        #listener = tf2_ros.TransformListener(self.tfBuffer) # 
        trans,code=self.get_robot_pose()
        if code==-1:
            rospy.logerr('transform between /robot1_tf/odom_groundtruth and /kf_slam not found')
            return
                
        x_slam=trans[0]
        y_slam=trans[1]
        self.set_point=[data.x,data.y]
        ori=Control_command()
        ori.data=np.math.atan2(y_obj,x_obj)+np.pi/2
        forward=dot_command()
        self.forward_setpoint=[x_obj+x_slam,y_obj+y_slam]
        forward.x=self.forward_setpoint[0]
        forward.y=self.forward_setpoint[1]

        self.OC.get_setpoint(ori)
        self.PC.get_setpoint(forward)
        self.histereis=True
        
    
    def set_control_signal(self):
        """ this method is used to send the control signal to the robot
        Input:
            .- self.OC.command: orientation control signal
            .- self.PC.command: position control signal
        Output:
            .- vl: left wheel velocity
            .- vr: right wheel velocity
        """
        if np.math.sqrt(self.set_point[0]**2+self.set_point[1]**2)<0.1:
         # If the norm of the setpoint is less than 10 cm, then the robot will stop.         
            v=0
        else:
            v=self.PC.command

        w=self.OC.command
        l=self.OC.base_line
        vr_w=w*l/2
        vl_w=-w*l/2
        # First turn and then go forward
        # if the velocity of orientation is too low, then you can apply the velocity of forward controller
        ori_err=self.OC.PID.y[0] # orientation error
        if self.histereis and abs(ori_err)<0.017*5: # 5 degree
            self.histereis=False
            vr=vr_w+v
            vl=vl_w+v  
        elif not self.histereis:
            vr=vr_w+v
            vl=vl_w+v 
        elif  not self.histereis and abs(ori_err)>0.017*10: # 5 degree
            self.histereis=True
            vr=vr_w
            vl=vl_w
        else:
            vr=vr_w
            vl=vl_w
        
        #elif not self.histereis and abs(vr)>0.15:
        #    self.histereis=True

        #if abs(vr)<0.1:
        #    vr=w*l/2+v
        #    vl=-w*l/2+v  

        #check velocity limits
        #if np.sqrt((vr**2+vl**2))>0.42:#0.42 is sqrt(2*(0.3m/s)^2). 0.3 Is the maximun velocity of the robot
        #    vr=vr*0.42/np.sqrt((vr**2+vl**2))
        #    vl=vl*0.42/np.sqrt((vr**2+vl**2))

        vel_max=0.3 # Constant
        if abs(vr)>vel_max or abs(vl)>vel_max:
            r=abs(np.max([vr,vl]))/vel_max
            vr=vr/r;vl=vl/r
        
        #Send control signal to PID of wheels
        self.OC.wheels_velocity_l.publish(vl/self.OC.wheel_radius)
        self.OC.wheels_velocity_r.publish(vr/self.OC.wheel_radius)

    def get_robot_pose(self):
        #try:
        #    #(trans,rot) = self.tf_listener.lookupTransform('/robot1_tf/odom_groundtruth', '/kf_slam', rospy.Time(0))
        #    #self.PC.pose=[trans[0],trans[1]]
        #    (trans,rot) = self.tfBuffer.lookup_transform('robot1_tf/odom_groundtruth', 'kf_slam', rospy.Time())
        ##except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):            
        #    rospy.logerr('No transformation was found')
        #    #continue
        #import pdb; pdb.set_trace()
        
        #trans = self.tfBuffer.lookup_transform('robot1_tf/odom_groundtruth', 'robot1_tf/base_link', rospy.Time())# Only for debug proposes
        try:
            trans = self.tfBuffer.lookup_transform('robot1_tf/odom_groundtruth', 'kf_slam', rospy.Time())
        except:
            rospy.logerr('No transformation was found')
            return 0,-1

        pos=[trans.transform.translation.x,trans.transform.translation.y]
        return pos,0

    def get_err_direction(self):
        """ this method is used to get the error direction
        Input:
            .- self.setpoint: setpoint
            .- self.PC.pose: robot position
        Output:
            .- err_direction: error direction
        """
        vec_robot=[np.math.cos(self.OC.orientation),np.math.sin(self.OC.orientation)]
        vec_err=[self.forward_setpoint[0]-self.PC.pose[0],self.forward_setpoint[1]-self.PC.pose[1]]
        err_direction=np.math.atan2(vec_err[1],vec_err[0])-np.math.atan2(vec_robot[1],vec_robot[0])
        return np.sign(np.math.cos(err_direction)), err_direction
        
    def run(self):
        time.sleep(10)  
        rospy.init_node('master_controller',anonymous=True)
        #self.tf_listener = tf.TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)       
        self.time=0.1
        self.rate = rospy.Rate(1/self.time) #hz    
        time.sleep(1)    # This is for fill the buffer of TF
        tran,code=self.get_robot_pose()
        while code==-1:
            tran,code=self.get_robot_pose()
            time.sleep(1)
        self.get_setpoints(data=dot_command(x=0,y=0)) # set the setpoint to the current position 
        while not rospy.is_shutdown():
            
            self.block_callback=True
            # main code is here
            tran,code=self.get_robot_pose()
            if code==-1:
                continue
            self.PC.pose=[tran[0],tran[1]]
            self.OC.compute_u()
            #self.OC.send_control_signal()
            self.PC.sign,_=self.get_err_direction()
            self.PC.compute_u()
            self.set_control_signal()
            #self.PC.send_control_signal()
            # end main code
            self.block_callback=False
            #print("set point",self.set_point,"command control :",self.PC.command,'pose slam: ',self.PC.pose,'controller: ',self.PC.set_point,'error: ',self.PC.error)
            #print("direccion del error: ",self.get_err_direction())
            self.debug.publish(Float64(data=self.PC.PID.y[0]))
            self.rate.sleep()
            
            pass
        
class Orientation:
    """ This class is used to compute the orientation of the robot and solve the problem of the singularity of the atan2 function
    
    To do this the class implements a flag variable that is changed when the robot is close to the singularity region.
    This method is like a hysterisis method. 
    """
    def __init__(self):
        self.sign=1
        
    def get_orientation(self,x,y=None):
        if y!=None: # if y is not None then x is the angle
             fi=np.math.atan2(y,x)
        else: # if y is None then x is an angle, and need to be converted to the same format 
            x2=np.math.cos(x)
            y=np.math.sin(x)
            fi=np.math.atan2(y,x2)
             
        if fi<-np.pi/2 and self.sign>0:
            fi=fi+2*np.pi
            return fi
        if fi>np.pi/2 and self.sign<0:
            fi=fi-2*np.pi
            return fi
        if abs(fi)<=np.pi/2:
            self.sign=1*np.sign(fi)
            return fi
        
        return fi
            
class NL_controller():
    def __init__(self):
        dot_command_topic_name='/dot_command'
        self.dot_command_topic=rospy.Subscriber(dot_command_topic_name,dot_command,self.get_setpoints,queue_size=1)
        self.err_distance_pub=rospy.Publisher('/err_distance',Float64,queue_size=1)
        self.debug=rospy.Publisher('/debug',Float64,queue_size=1)
        #max velocity
        self.v_mx=0.3
        self.w_mx=2*np.pi/4# a complete rotation in 4 seconds
        self.kv=self.v_mx
        self.kwa=(abs(self.w_mx)-abs(self.kv)/2)/np.pi
        self.OC=Orientation_controller()
        self.set_point=[0,0]
        self.ori_robot=Orientation()
        self.ori_setpoint=Orientation()
        self.minimum_distance=0.3

    def get_err_direction(self):
        """ this method is used to get the error direction
        Input:
            .- self.setpoint: setpoint
            .- self.PC.pose: robot position
        Output:
            .- err_direction: error direction
        """
        vec_robot=[np.math.cos(self.OC.orientation),np.math.sin(self.OC.orientation)]
        vec_err=[self.forward_setpoint[0]-self.PC.pose[0],self.forward_setpoint[1]-self.PC.pose[1]]
        err_direction=np.math.atan2(vec_err[1],vec_err[0])-np.math.atan2(vec_robot[1],vec_robot[0])
        return np.sign(np.math.cos(err_direction)), err_direction
        
    def update_state(self):
        # get the robot pose, calculate the error ro and alpha
        #import pdb;pdb.set_trace()
        trans,code=self.get_robot_pose()
        if code==-1:
            rospy.logerr('transform between /robot1_tf/odom_groundtruth and /kf_slam not found')
            return
        x=trans[0];x_sp=self.set_point[0]
        y=trans[1];y_sp=self.set_point[1]
        self.ro=np.math.sqrt((x_sp-x)**2+(y_sp-y)**2)
        fi=self.ori_setpoint.get_orientation(x_sp-x,y_sp-y)
        robot_fi=self.ori_robot.get_orientation(self.OC.orientation)        
        self.alpha=fi-robot_fi
        if abs(self.alpha)>np.pi:
            self.alpha=self.alpha-2*np.pi*np.sign(self.alpha)

        #print("orientation: ", robot_fi,"fi_setpoint: ",fi,"angle error: ",self.alpha," radial err:",self.ro)

        
        # calculate the error direction trought angle between vectors
        #x_sp=np.math.cos(fi);y_sp=np.math.sin(fi)
        #x_me=np.math.cos(self.OC.orientation); y_me=np.math.sin(self.OC.orientation)
        #err=np.math.acos(x_sp*x_me+y_sp*y_me)# -np.pi/2 # -pi/2 is for avoid oscilations  
        #err2=-self.OC.orientation+fi
        #fi_humber=np.math.atan2(y_sp-y,x_sp-x)
        #err=err*np.sign(self.OC.orientation-fi)
        #fi=self.OC.orientation
#
        #R=np.array([[np.math.cos(fi),-np.math.sin(fi)],[np.math.sin(fi),np.math.cos(fi)]])
        #v=np.array([[x_sp-x],[y_sp-y]])
        #vr=R@v
        ##if abs(err2)>np.pi:
        ##    err2=err2-np.sign(err2)*2*np.pi
        ##dis=np.math.sqrt((x_sp-x_me)**2+(y_sp-y_me)**2)
        ##if dis> np.math.sqrt(2):# if the distance between setpoint and orientation is greater than 90 degrees set the error to pi/2
        ##    err=np.pi/2 *np.sign(err)
#
        #print("orientation: ", self.OC.orientation,"fi: ",fi,"angle error: ",err," radial err:",self.ro,'ori: ',self.OC.orientation)
#
        #self.alpha=err

        #vec_robot=[np.math.cos(self.OC.orientation),np.math.sin(self.OC.orientation)]
        #vec_err=[x_sp-x,y_sp-y]
        #self.alpha=np.math.atan2(vec_err[1],vec_err[0])-np.math.atan2(vec_robot[1],vec_robot[0])
        #self.alpha=np.math.atan2(y_sp-y,x_sp-x)-np.pi/2
        

    def get_setpoints(self,data):
        x_obj=data.x
        y_obj=data.y   
        trans,code=self.get_robot_pose()
        if code==-1:
            rospy.logerr('transform between /robot1_tf/odom_groundtruth and /kf_slam not found')
            return
                
        x_slam=trans[0]
        y_slam=trans[1]
        self.set_point=[x_obj+x_slam,y_obj+y_slam]
        

    def get_control_signals(self):
        # get the control signals, pag. 101 Msc. Secchi 1998
        kv=self.kv; kwa=self.kwa # for readability purposes
        alpha=self.alpha; ro=self.ro # for readability purposes
        v=kv*np.math.tanh(ro)*np.math.cos(alpha)
        w=kwa*alpha+kv*np.math.tanh(ro)*np.math.sin(alpha)*np.math.cos(alpha)/ro
        return v,w
    
    def apply_control_signals(self):
        """ calculate and publish the control signals to the PID of wheels
        call get_control_signals function, get fordward and angular velocity, calculate velocity for each wheel and publish it
        """
        v,w=self.get_control_signals()
        l=self.OC.base_line # for readability purposes
        vr=v+w*l/2
        vl=v-w*l/2
        self.OC.wheels_velocity_l.publish(vl/self.OC.wheel_radius)
        self.OC.wheels_velocity_r.publish(vr/self.OC.wheel_radius)
    
    def get_robot_pose(self):
        #try:
        #    #(trans,rot) = self.tf_listener.lookupTransform('/robot1_tf/odom_groundtruth', '/kf_slam', rospy.Time(0))
        #    #self.PC.pose=[trans[0],trans[1]]
        #    (trans,rot) = self.tfBuffer.lookup_transform('robot1_tf/odom_groundtruth', 'kf_slam', rospy.Time())
        ##except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):            
        #    rospy.logerr('No transformation was found')
        #    #continue
        #import pdb; pdb.set_trace()
        
        #trans = self.tfBuffer.lookup_transform('robot1_tf/odom_groundtruth', 'robot1_tf/base_link', rospy.Time())# Only for debug proposes
        try:
            trans = self.tfBuffer.lookup_transform('robot1_tf/odom_groundtruth', 'kf_slam', rospy.Time())
        except:
            rospy.logerr('No transformation was found')
            return 0,-1

        pos=[trans.transform.translation.x,trans.transform.translation.y]
        return pos,0
    
    def run(self):
        time.sleep(10)  
        rospy.init_node('No_linear_controller',anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)       
        self.time=0.1
        self.rate = rospy.Rate(1/self.time) #hz    
        time.sleep(1)    # This is for fill the buffer of TF
        code=-1
        while code==-1:
            tran,code=self.get_robot_pose()
            time.sleep(1)
        self.get_setpoints(data=dot_command(x=0,y=0)) # set the setpoint to the current position 
        while not rospy.is_shutdown():
            self.update_state()
            if abs(self.ro)>self.minimum_distance: # ro can't be zero
                self.apply_control_signals()
            else:
                self.OC.wheels_velocity_l.publish(0)
                self.OC.wheels_velocity_r.publish(0)

            self.err_distance_pub.publish(Float64(data=self.ro))
            self.debug.publish(Float64(data=self.alpha))            
            self.rate.sleep()        

def main():
    #asd=Orientation_controller()
    #asd.run()
    #asd=Master_Controller()
    #asd.run()
    asd=NL_controller()
    asd.run()
    
    
if __name__=='__main__':
    try:
        main()         
    except rospy.ROSInterruptException:
        pass
    
    