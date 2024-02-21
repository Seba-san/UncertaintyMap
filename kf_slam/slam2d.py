#!/usr/bin/env python3
"""
This code read LIDAR and orientation topics. Procesess data and make a slam 

leer datos y procesarlos
administracion de landmarks:
    .- matchear
    .- agregar nuevos
realizar el procesamiento del KF 
ploteo del mapa
computo de los mapas de exploracion
planificacion

"""

import rospy
from sensor_msgs.msg import LaserScan as lasermsg
#from std_msgs.msg import Float32 as wheel_velocity_msg
from std_msgs.msg import Float64 as wheel_velocity_msg
import numpy as np
from sklearn.cluster import KMeans
from KF import KF # KF2 is for python 2.7, KF is for 3.7 and newer
from gazebo_msgs.msg import ModelStates as models
import tf
from std_msgs.msg import Float32MultiArray as states_
from std_msgs.msg import MultiArrayDimension 

import tf2_ros
from  tf2_msgs.msg import TFMessage
from  geometry_msgs.msg import TransformStamped
import time
from copy import deepcopy as copy
import time
#from gazebo_manager import GazeboSimulationController

class Slam2d_base:
    def __init__(self):       
        self.pose=np.array([0,0],dtype='f') # x,y
        self.angles=np.linspace(-np.pi,np.pi,int(1+np.pi*2/0.008738775737583637))
        Q_noise=0.01 # OJO ACA, se setea manualmente, pero en la realidad se usa rosparam. Hay un bug entre rosparam y pool de procesos...
        R_noise=0.01
        self.kf=KF(R_noise=R_noise,Q_noise=Q_noise)# R is laser noise and Q is controller and state noise (10% approx)
        # Initial values of covariance matrix
        self.kf.P[0,0]=0.01
        self.kf.P[1,1]=0.01
        self.block_callback=False
        self.new_measurement=False

        self.cones=np.array([],dtype='f').reshape(0,2)
        self.time=0.1 # se usa? Se vuelve a definir en la otra clase y se sobreescribe... esta medio al dope aca parece.

    def build_vector_states(self):
        """
        build a vector with states vector and covariance matrix.
        The first list of vector is states vector and covariance is the next ones.
        """
        data=states_()
        #data.layout.dim # list
        #data.layout.data_offset # int

        N=self.kf.x.shape[1]
        aux=self.kf.x[0,:].tolist()
        data.data=aux
        for i in range(N):
            p_row=self.kf.P[:,i].tolist()
            data.data.extend(p_row)

        data.layout.dim=[MultiArrayDimension(label='row',size=N,stride=1)]
        self.states_sended=data
                 
class SLAM2D(Slam2d_base):
    def __init__(self):
        super().__init__()
       
        Q_noise=rospy.get_param('Q_noise',0.01)
        R_noise=rospy.get_param('R_noise',0.01)
        self.kf=KF(R_noise=R_noise,Q_noise=Q_noise)
        self.kf.P[0,0]=rospy.get_param('P_0',0.01)
        self.kf.P[1,1]=rospy.get_param('P_0',0.01)
        self.threshold_new_landmark=rospy.get_param('max_distance_landmark',5.0)# 5.0#0.5 # max distance to consider a landmark as new
        

        print('Iniciando slam')
        rospy.init_node('kf_slam_node')
        topic_laser='/robot1/laser/scan_lidar_horizontal'
        self.laser_subs=rospy.Subscriber(topic_laser,lasermsg,self.laser_callback,queue_size=1)
        self.laser__slam_pub=rospy.Publisher('/slam/laser',lasermsg,queue_size=1)
        topic_robot_state='/gazebo/model_states'
        self.orientation_sub=rospy.Subscriber(topic_robot_state,models,self.get_orientation,queue_size=1)

        #topic_left_wheel='/robot1/left_dc_motor/command'
        topic_left_wheel='/robot1/left_wheel_controller/command' # commando del path_follower
        self.wheels_velocity_l=rospy.Subscriber(topic_left_wheel,wheel_velocity_msg,self.get_velocity_l,queue_size=1)
        #topic_right_wheel='/robot1/right_dc_motor/command'
        topic_right_wheel='/robot1/right_wheel_controller/command'
        self.wheels_velocity_r=rospy.Subscriber(topic_right_wheel,wheel_velocity_msg,self.get_velocity_r,queue_size=1)

        topic_states_covariance='/slam/states_cova'
        self.states_cova=rospy.Publisher(topic_states_covariance,states_,queue_size=1)
        self.first_time=True

        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)
        self.n_landmarks_laser=0

        # TODO: los siguientes parametros no se usan, revisar.
        self.wheel_radious_l=0.108 # TODO: get from robot params
        self.wheel_radious_r=0.108
        self.base_line=0.34
        self.lineal_vel_l=0
        self.lineal_vel_r=0
        self.orientation=0.0

        self.std_max=0.5 #maximum stardard deviation of a landmark in laser frame
        #self.std_max=4.0
        self.t0=rospy.get_time() # para obtener las acciones de control
        
    def send_tf_slam(self):
        # Source http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29
        msg=TransformStamped()
        msg.header.frame_id='robot1_tf/odom_groundtruth'
        msg.header.stamp=rospy.Time.now()
        msg.child_frame_id='kf_slam'
        msg.transform.translation.x=self.kf.x[0,0]
        msg.transform.translation.y=self.kf.x[0,1]
        msg.transform.rotation.w=1.0
        tff=TFMessage([msg])
        self.pub_tf.publish(tff)
    
    def get_velocity_l(self,data=wheel_velocity_msg):
        self.lineal_vel_l=data.data*self.wheel_radious_l
        
    def get_velocity_r(self,data=wheel_velocity_msg):
        self.lineal_vel_r=data.data*self.wheel_radious_l

    def get_orientation(self,data=models):
        """
        get orientation from gazebo and calculate rotation
        Input:
            .- gazebo model
        Output:
            .- self.orientation: Orientation of robot in radians
        """
        if self.block_callback:
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

    def get_control_signal(self):
        """
        Convert from angular velocity of wheels to delta position 
        """
        vl=self.lineal_vel_l
        vr=self.lineal_vel_r
        r1=self.wheel_radious_l
        r2=self.wheel_radious_r
        t=rospy.get_time()
        pose=(t-self.t0)*(vl+vr)/2
        x=pose*np.math.cos(self.orientation)
        y=pose*np.math.sin(self.orientation)
        self.t0=t+0.0
        return np.array([[x,y]]).T    
    
    def laser_callback(self,data=lasermsg()):
        # only store data if is not blocked and send laser data to laser_slam topic
        data_=copy(data)
        fi=self.orientation+np.pi
        fi=np.mod(fi + np.pi, 2 * np.pi) - np.pi
        data_.header.frame_id='kf_slam'
        data_.angle_max=np.max(fi)
        data_.angle_min=np.min(fi)
        
        self.laser__slam_pub.publish(data_)
        if self.block_callback:
            return
        self.new_measurement=True
        self.laser_data=data

                      
    def laser_process(self):
        """
        Read data from topic and apply segmentation to separate landmarks 
        variables affected:
        self.n_landmarks_laser: amount of cones viewed. If is 0, don't apply self.cones and self.assignation variables.
        self.cones: points of cones
        self.assignation: labels of points in self.cones

        k-means is very expensive in time. For 2 cones the routine take 0.1s. On the other hand, lidar data takes 0.04s (25Hz).
        """
        #data=lasermsg()
        #import pdb; pdb.set_trace()
        
        data=copy(self.laser_data)
        v_data=np.array(list(data.intensities))# Vector data
        idx_cone=v_data==1.0
        idx_occuped=v_data>=0.5
        ranges=np.array(list(data.ranges))
        cone_range=ranges[idx_cone]
        if cone_range.shape[0]==0:
            self.n_landmarks_laser=0
            self.cones=np.array([],dtype='f').reshape(0,2)
            self.assignation=0
            rospy.loginfo_once('no landmarks at range')          
            return
        
        cone_angle=self.angles[idx_cone]
        cone_x=cone_range*np.cos(cone_angle)# red axes is x
        cone_y=cone_range*np.sin(cone_angle)# green axes is y
        cones=np.array([cone_x,cone_y]).T
        std_cones=np.sqrt(np.var(cone_x)+np.var(cone_y))
        i=1 # amount of cones
        
        if std_cones>self.std_max:# multiple cones
            i=2            
            asd=0
            while asd==0:
                #KMeans(n_clusters=i,)
                #assing,centers=k_means(cones,i) # inpunt: points, amount of clusters, output: labels, centers
                kmeans = KMeans(n_clusters=i,n_init='auto')
                kmeans.fit(cones)
                assign = kmeans.labels_
                #centers = kmeans.cluster_centers_

                asd=1
                i=i+1
                for k in range(i-1):
                    idx=assign==k                    
                    tot_var=np.sqrt(np.var(cones[idx,0])+np.var(cones[idx,1])) #variancia total
                    if tot_var>self.std_max:
                        asd=asd*0    # iterate again with more clusters         
            self.n_landmarks_laser=i-1      # i-1 cause was added 1 without computation             
            #import pdb; pdb.set_trace()
            self.assignation=assign      
            
        else:
            #Only one cone was viewed
            self.n_landmarks_laser=i
            n=cones.shape[0]
            self.assignation=np.zeros(n)
        self.cones=cones
        #print(self.n_landmarks_laser," conos encontrados")
        
    def find_correspondences(self):
        """
        This function obtain which landmark view corresponds to landmarks in vector states. Moreover this function get
        which measure is a new landmark or not.
        Inputs: 
            .- measures
        Outputs:
            .- index association:
                self.matching:  [state vector, measuerement]
                self.new_landmarks_n: amount of new landmarks
                self.cones_centers: centers of all cones in world frame
        """
        """
        Se plantea un macheo a lo cabeza, aun falta implementar
        """
        loc=self.kf.x[0][0:2]
        #loc=self.kf.x[0:2]
        landmarks=self.kf.x[0][2:]
        landmarks=np.reshape(landmarks,(-1,2))
        #n_landmarks=int(len(landmarks)/2)
        n_landmarks=int(len(landmarks))
        #print(n_landmarks)
        #import pdb; pdb.set_trace()     
        cc=np.math.cos(self.orientation)
        ss=np.math.sin(self.orientation)
        rotation_matrix=np.array([[cc,-ss],[ss,cc]],dtype='f').T      
        cones_world=np.matmul( self.cones , rotation_matrix)+loc # from laser frame to world frame 
        #cones_world= self.cones @ rotation_matrix+loc # from laser frame to world frame 
        self.new_landmarks_n=0
        #self.matching=np.array([[]])
        self.matching=np.array([],dtype='f').reshape(0,2)
        self.new_matching=np.array([],dtype='f').reshape(0,2)
        
        self.cone_centers=np.array([],dtype='f').reshape(0,2)
        #import pdb; pdb.set_trace()  
        for i in range(self.n_landmarks_laser):
            idx=self.assignation==i
            cone=np.mean(cones_world[idx],axis=0)           
            diff=cone -landmarks
            #import pdb; pdb.set_trace()
            #if diff.shape[0]==0:
            #    match=np.linalg.norm(diff,axis=1) # first run
            #else:
            #    import pdb; pdb.set_trace()
            #    match=np.math.sqrt(np.reshape((cone -landmarks),(1,2*n_landmarks)) @ np.linalg.inv(self.kf.P[2:,2:]) @ np.reshape((cone -landmarks),(2*n_landmarks,1))) #mahalanobis distance
            match=np.linalg.norm(diff,axis=1) # first run
            if match.shape[0]==0:
                # First run
                match=np.array([10,10],dtype='f')
            idx=np.argmin(match)
            if match[idx]<self.threshold_new_landmark: # TODO: umbral de aceptacion
                # asociate the matching                                       
                self.matching=np.r_[self.matching, np.c_[idx,i]]                
            else:
                # add new landmark
                self.new_landmarks_n=self.new_landmarks_n+1
                idx_=self.new_landmarks_n+n_landmarks-1
                self.matching=np.r_[self.matching, np.c_[idx_,i]]
                self.new_matching=np.r_[self.new_matching, np.c_[idx_,i]]
            cone_=cone
            cone_=cone_[:,np.newaxis].T
            self.cone_centers=np.r_[self.cone_centers, cone_]
            if np.isnan(self.cone_centers).any():
                print('hay valores NAN revisar!')
                import pdb;pdb.set_trace()

        
    def add_new_landmarks(self):
        """
        Add new landmarks to states and covariance matrix
        Modify matrices:
        .- self.kf.A
        .- self.kf.B 
        .- self.kf.Q
        .- self.kf.P
        .- self.kf.x 

        Also works when there aren't landmarks at range. 
        """
        cc=np.math.cos(self.orientation)
        ss=np.math.sin(self.orientation)
        rotation_matrix=np.array([[cc,-ss],[ss,cc]],dtype='f').T
        cones_world=self.cones@rotation_matrix+self.kf.x[0][0:2] # from laser frame to world frame  
        #Uptate A matrix of kalman
        nn=self.kf.A.shape[0]
        self.kf.A=np.eye(nn+self.new_landmarks_n*2)
        #Update B matrix of kalman        
        self.kf.B=np.r_[self.kf.B,np.zeros((self.new_landmarks_n*2,2))]
        #Update Q matrix of kalman
        nn=self.kf.A.shape[0]
        self.kf.Q=np.zeros((nn,nn),dtype='f')
        self.kf.Q[0:2,0:2]=self.kf.Q_
        assignation=self.assignation        
        for i in self.new_matching[:,1]:
            idx=assignation==(i) # warning, this variable change in interruption
            cone=np.mean(cones_world[idx],axis=0)        
            cone=cone[:,np.newaxis] # para agregarle una nueva dimension
            self.kf.x=np.c_[self.kf.x,cone.T] # add new state
            Pll_new=self.kf.P[:2,:2]+self.kf.R_
            Pxl_new=self.kf.P[:2]
            self.kf.P=np.r_[self.kf.P,Pxl_new]
            cn=np.r_[Pxl_new.T,Pll_new]
            self.kf.P=np.c_[self.kf.P,cn]  
                        
    def run(self):
        
        self.tf_listener = tf.TransformListener()
        self.time=0.5 # seconds
        self.rate = rospy.Rate(1/self.time) #hz
        start=time.time()
        print('Iniciando nodo')        

        while not rospy.is_shutdown():    

            try:
                (trans,rot) = self.tf_listener.lookupTransform('/robot1_tf/odom_groundtruth', '/robot1_tf/base_link', rospy.Time(0))
                self.calculate_orientation([rot[-2],rot[-1]])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # If there is a tf message, do not run. Fix this...
                continue

            if self.first_time:
                #import pdb; pdb.set_trace()
                self.kf.x[0,0]=trans[0]
                self.kf.x[0,1]=trans[1]
                self.first_time=False
            
            self.block_callback=True # Avoid that some variables changes
            if not self.new_measurement: # Solo para fines de testeo $1!!
                self.block_callback=False
                self.rate.sleep() 
                continue
            else:
                self.new_measurement=False

            u_signal=self.get_control_signal()
            if time.time()-start>5.0 and np.abs(u_signal[0])<0.01 and np.abs(u_signal[1])<0.01: # Si las signals de control son pequeñas, no se actualiza el filtro
                # Por las condiciones iniciales que necesita convergencia, se actualiza durante los 5 primeros segundos.
                rospy.logwarn_once('Control signal is too small')
            #    self.kf.Q_=np.array([[0,0],[0,0]],dtype='f')
            #else:
            #    self.kf.Q_=self.Q

                # Experimental
                self.block_callback=False
                self.build_vector_states()
                self.states_cova.publish(self.states_sended)
                self.send_tf_slam()
                self.rate.sleep() 
                continue


                
            self.laser_process()
            self.find_correspondences()   
            if self.new_landmarks_n>0 or self.cones.shape[0]==0: 
                #import pdb; pdb.set_trace()
                self.add_new_landmarks()   
            
            #import pdb; pdb.set_trace()
            #self.matching
            #if np.max(self.matching) >1:
            #    pass
                #print('Asigna un matching sin tener nuevos landmarks')
                #import pdb; pdb.set_trace()

            self.kf.update_matrix(self.matching)
            #print('H matrix :',self.kf.H)
            #import pdb; pdb.set_trace()
            
            self.kf.update(u=u_signal,z=self.cone_centers.reshape(-1,1))
            #if  np.isnan(self.kf.x[0]).any():
            #    print('Ocurrio un error poco usual, revisar... hay variables NAN.')
            #    import pdb; pdb.set_trace()
            
            #print('X loc: ', self.kf.x[0][:2])
            #print('X loc: ', self.kf.x[0])
            rospy.loginfo('X loc: '+ str(self.kf.x[0]))
            #print(' u_signal: ',u_signal.T)
            #print('diferencia con tf: ',[self.kf.x[0][0]-trans[0],self.kf.x[0][1]-trans[1]])
            #print('transformacion ', trans)
            P=self.kf.P
            rospy.loginfo('variance xx and yy of robot '+ str([P[0,0],P[1,1] ])+' variance last landmark '+ str([P[-2,-2],P[-1,-1]]))

            self.block_callback=False
            #import pdb; pdb.set_trace()
            self.build_vector_states()
            self.states_cova.publish(self.states_sended)
            self.send_tf_slam()
            self.rate.sleep()    

def main():
    """
    get measures
    find correspondences
    uptate matrices R and H
    if corresponds: add new landmarks
    run KF
    calculate the maps: exploration map and landmarks map (and occupation map?)
    obtain next control action
    """
    pass

if __name__ == '__main__':    
    try:
        time.sleep(3)
        test=SLAM2D()
        test.run()

    except rospy.ROSInterruptException:
        pass
