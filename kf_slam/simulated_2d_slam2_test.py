#!/usr/bin/env python3

from KF import KF
import numpy as np
from std_msgs.msg import Float32MultiArray as states_
import rospy
from slam2d import Slam2d_base
import tf2_ros
import geometry_msgs.msg
from copy import deepcopy as copy
import sys
import time
from map_server import MapServer
from nav_msgs.msg import OccupancyGrid as map_msg
from multiprocessing import Pool, cpu_count,TimeoutError

from uncertainty_frointier import UFrontier
from insert_markers import RvizMarkerPublisher

from gazebo_manager import GazeboSimulationController
from simulated_2d_slam_fast import FastEstimation

from GET_data import GET_data
from nav_msgs.msg  import Path

class SLAM2D_simulated(Slam2d_base):
    """make a simulated slam2d and a local map to get divergence of each trajectory.

    This class is the core of the simulation. through Herencia get all the methods of Slam2d_base class.
    In a internal variable called self.local_map is stored MapServer object to administrate
    exploration and landmarks map. At the end of the simulation the divergence and best control signal
    is provided.
    
    This class has two methods:
    .- run(): run the simulation and store the local map
    .- _run(): do the same that run() but publish in /slam_simulated/states and the map in /simulation/map. For 
    do this you need to set debug=True in the constructor.
    
    TODO: the most expensive computational cost is in SLAM and map server. 
    Maybe is better to use another language to do this, like golang or c++.
    """
    def __init__(self,kalman_object=KF(),control_signal=np.array([[0,0]]),debug=False,parameters=[]):
        """ Create a simulated kf-slam 
        inputs:
        - Kalman object with variables setted
        - Control signal that will be repeated

        TODO: No me termina de cuadrar dentro de un nodo, quizas sea solo un servicio
        u otra forma de llamarlo
        """
        super().__init__()
        if debug:            
            self.states_topic='/slam_simulated/states'
            self.states_cova=rospy.Publisher(self.states_topic,states_,queue_size=1)  
            self.map_topic=rospy.Publisher('/simulation/map',map_msg,queue_size=1)
            
        self.u_controls=control_signal  
        #self.kf=copy(kalman_object)  
        self.FOV=5.0 #m TODO: make a function or object with FOV methods  
        self.new_data=False  
        # Local map administration
       
        self.local_map=MapServer(sb=parameters[4],fov=parameters[5],cell_size=parameters[6])        
        self.local_map.exploration_flag=False # update exploration map without max function
        #self.local_map.K_update=1.0# $1 OJO ACA!!! se cambia el rate de actualizacion
        self.collision_detected=False
        # self.minimum_distance=1.8 # minimun distance to obstacle for avoid collision
        self.minimum_distance=parameters[7] # minimun distance for last point of path.
        self.threshold_obstacle=0.3 # threshold to considerate an object as a collision; 0.5 is a unknown space.
        self.last_obst_dist=0.0 # last distance to obstacle
        self.continue_simulation=False # if is true the simulation continue from last point
        self.step_factor=2# is the ratio between the step of the simulation and the step of halucination
        self.step_simulation=0.15# m/s
        self.step_halucination=self.step_simulation*self.step_factor
                
        #self.fe=FastEstimation(self.local_map.exploration_map,self.kf.P,self.kf.x,self.FOV,
        #                            [self.local_map.cell_size,np.array([300,300]),self.kf.Q_[0,0],self.local_map.beta,self.local_map.a],
        #                            self.local_map.obstacle_map)# [cellsize,center,Q0]
        self.fix_parameters(parameters)
    
    def fix_parameters(self,parameters):
        # Para poder paralelizar hay que hacer esto, no se peude acceder desde rosparams
        self.kf.P[0,0]=parameters[0]
        self.kf.P[1,1]=parameters[0]
        self.kf.R_=np.array([[1,0],[0,1]],dtype='f').dot(parameters[1]) # measure noise
        self.kf.Q_=np.array([[1,0],[0,1]],dtype='f').dot(parameters[2])*self.step_factor # ojo aca! $1
        self.kf.Q=self.kf.Q_
        self.threshold_new_landmark=parameters[3]

    # No se usa esta funcion me parece
    def get_states(self,data=states_()):
        """ Build states and covariance matrix P from Float32MultiArray topic
        First data is a state vector and the follow one is an covariance matrix
        """
        if self.new_data:
            return # only enter once
        N=data.layout.dim[0].size
        self.P=[]
        self.states=list(data.data[0:N])
        #self.states_0=list(data.data[0:N])
        for i in range(N):
            i_0=(i+1)*N
            i_1=(i+2)*N
            self.P.append(list(data.data[i_0:i_1]))

        self.new_data=True
                
    def fake_sanesor(self, states=np.array([[]])):
        """ read states and covariance matrix for generate landmakrs measures
        Tiene da dar las distancias relativas al robot basandose en los estados y el FOV
        self.matching:  [state vector, measuerement]
        self.cones_centers: centers of all cones viewed in world frame
        
        This function was checked for 2 landmarks or less
        """
        #import pdb;pdb.set_trace() #$1
        # tomar todos los datos de los estados
        loc=self.kf.x[0][0:2]+self.u_control.T[0]; loc=loc[:,np.newaxis].T
        landmarks=self.kf.x[0][2:]
        landmarks=np.reshape(landmarks,(-1,2))
        # calcular la distancia a todos

        diff=loc -landmarks
        match=np.linalg.norm(diff,axis=1)
        # seleccionar los  cercanos 
        match=match<self.FOV
        states_match=np.where(match)[0].astype(int)
        N=states_match.shape[0]
        measure=landmarks[match]#+np.random.normal(loc=0.0,scale=self.kf.R_[0,0],size=(N,2))
        measure_match=np.linspace(0,N-1,N).astype(int)
        # obtener las medidas relativas
        # obtener el matching de esas medidas 
        #np.r_[self.matching, np.c_[idx,i]]   
        self.matching=np.c_[states_match,measure_match]
        self.cone_centers=measure

        #self.cone_centers=np.array([[]])
        #self.matching=np.array([[]])
        #self.matching=np.array([],dtype='f').reshape(0,2)
        #self.cone_centers=np.array([],dtype='f').reshape(0,2)
        return 
    
    def get_control_signal(self):
        """
        Tiene que interpolar los puntos en el espacio???
        toma las acciones de control y las repite
        """
        x=self.u_control[0,0]
        y=self.u_control[0,1]
        return np.array([[x,y]]).T  

    def send_tf_simulator(self):
        """
        Solo para debugear, enviar el dato simulado
        """
        t = geometry_msgs.msg.TransformStamped()
        #import pdb;pdb.set_trace()
        t.child_frame_id=rospy.get_name()
        #t.header.frame_id='kf_slam'
        t.header.frame_id='robot1_tf/odom_groundtruth'
        t.header.stamp=rospy.Time.now()         
        t.transform.translation.x=self.kf.x[0,0]
        t.transform.translation.y=self.kf.x[0,1]
        t.transform.rotation.w=1.0
        self.br.sendTransform(t) 
    
    def build_kalman_matrices(self):
        """ update kalman matrices
        update kalman matrices like add_new_landmarks function of SLAM2D class
        """
        self.kf.x=np.array([self.states])
        self.kf.P=np.array(self.P)
        # From add_new_landmarks function
        new_landmarks_n=self.kf.x[0].shape[0]-2
        nn=self.kf.A.shape[0]
        self.kf.A=np.eye(nn+new_landmarks_n)
        #Update B matrix of kalman        
        self.kf.B=np.r_[self.kf.B,np.zeros((new_landmarks_n,2))]
        #Update Q matrix of kalman
        nn=self.kf.A.shape[0]
        self.kf.Q=np.zeros((nn,nn),dtype='f')
        self.kf.Q[0:2,0:2]=self.kf.Q_
                       
    def run_(self):
        """This function run the simulation for each control signal
         ROS is not required. 
        """                   
        if not self.continue_simulation:            
            self.build_kalman_matrices() 
        N=len(self.u_controls[0])
        x0=[self.kf.x[0,0],self.kf.x[0,1]]
        self.last_obst_dist=1000.0
        colission_detected=False
        #self.traj_prob=[0,0]
        self.build_vector_states()
        self.local_map.full_data=self.states_sended
        self.local_map.run()
        if not self.continue_simulation: 
             self.fe=FastEstimation(self.local_map.exploration_map,self.kf.P,self.kf.x,self.FOV,
                                    [self.local_map.cell_size,np.array([300,300]),self.kf.Q_[0,0],self.local_map.beta,self.local_map.a],
                                    self.local_map.obstacle_map)# [cellsize,center,Q0]
      
        for k in range(N):
            u=[self.u_controls[0][k],self.u_controls[1][k]]
            self.u_control=np.array([[u[0],u[1]]]).T  
            #self.u_control=u
            #u_signal=np.array([[u[0],u[1]]]).T  
            #self.fake_sanesor()  
            #self.kf.update_matrix(self.matching)    
            #self.kf.update(u=self.u_control,z=self.cone_centers.reshape(-1,1))
            ## make a ROS message and then pass data by memory (next lines)
            #self.build_vector_states()
            #self.local_map.full_data=self.states_sended # this is the way to pass data by memory
            #self.local_map.run()
            #self.traj_prob[0]=self.traj_prob[0]+self.probability_trajectory()
            #self.traj_prob[1]=self.traj_prob[1]+1
            if k==0:
                #self.initial_divergence=self.local_map.get_divergence()
                self.initial_divergence=self.fe.get_M_update(self.kf.x)

            #self.pondered_divergence(k)
            self.kf.x[0][0]=self.kf.x[0][0]+self.u_control[0][0]
            self.kf.x[0][1]=self.kf.x[0][1]+self.u_control[1][0]
            diver=self.fe.get_M_update(self.kf.x)
            if self.avoid_collision():
                print('Colision detectada')
                colission_detected=True
                break

        if colission_detected:            
            bias=0.0
            new_u=[self.kf.x[0,0]-x0[0],self.kf.x[0,1]-x0[1]]

            # security range setted by bias variable
            norm=np.math.sqrt(new_u[0]**2+new_u[1]**2)
            if norm>bias:
                norm2=norm-bias
                new_u[0]=new_u[0]*norm2/norm
                new_u[1]=new_u[1]*norm2/norm
            
            self.collision_detected=True
            return True,new_u
        else:
            return False,[]
    
    def pondered_divergence(self,k):
        gamma=0.75        
        if k==0:
            self.last_diver=self.initial_divergence
            self.accumulated_diver=self.last_diver+0.0
        current_diver=self.local_map.get_divergence()
        delta_diver=current_diver-self.last_diver
        self.accumulated_diver=self.accumulated_diver+(gamma**k)*delta_diver
        self.last_diver=current_diver

    def run(self):    
        """This function run the simulation for each control signal
         ROS is not required. 
        """      

        if not self.continue_simulation:
            self.build_kalman_matrices()  

        #N=int(len(self.u_controls[0])-self.minimum_distance/self.step_halucination)# Se resta el ultimo tramo para que simule exactamente lo que se realiza
        N=len(self.u_controls[0])
        x0=[self.kf.x[0,0],self.kf.x[0,1]]
        self.last_obst_dist=1000.0
        colission_detected=False        
        self.build_vector_states()
        self.local_map.full_data=self.states_sended
        self.local_map.run() 
        
        for k in range(N):
            u=[self.u_controls[0][k],self.u_controls[1][k]]                        
            # make a loop for simulate the robot movement for each control signal
            self.u_control=np.array([[u[0],u[1]]]).T  
            self.fake_sanesor()               
            self.kf.update_matrix(self.matching)   
            self.kf.update(u=self.u_control,z=self.cone_centers.reshape(-1,1))
            self.build_vector_states()
            # make a ROS message and then pass data by memory (next lines)
            self.local_map.full_data=self.states_sended
            for i in range(int(self.step_factor)):
                self.local_map.run()                    
        return False,[]
                    
    def _run_debug(self):    
        """ This functions is for debuging purposes. 
        Do the same thing of run function but coneecting to a some topics.
        when you run this function, you need run another script to make the map, 
        as example:
        python map_server.py /slam_simulated/states robot1_tf/odom_groundtruth /slam/landmark_map2 /exp_map2
        however a topic called /simulation/map is published and you can visualize the map in rviz.        
        """           
        #rospy.init_node('simulation_node',anonymous=True)
        self.br = tf2_ros.TransformBroadcaster()                  
        self.time=1.0
        self.rate = rospy.Rate(1/self.time) #hz
        print('Iniciando simulacion en modo debug...')
        #import pdb;pdb.set_trace()
        #while not self.new_data: # wait for a states data
        #    pass  
        #import pdb;pdb.set_trace() 
        print('Capturada la info requerida, computando...')   
        if not self.continue_simulation:
            self.build_kalman_matrices()  
             
        # TODO: what about Q and R matrices?
        N=len(self.u_controls[0])
        #import pdb;pdb.set_trace()
        x0=[self.kf.x[0,0],self.kf.x[0,1]]
        self.last_obst_dist=1000.0
        colission_detected=False
        
        self.build_vector_states()
        self.local_map.full_data=self.states_sended
        self.local_map.run() 
        #self.fe=FastEstimation(self.local_map.exploration_map,self.kf.P,self.kf.x,self.FOV,
        #                            [self.local_map.cell_size,np.array([300,300]),self.kf.Q_[0,0],self.local_map.beta,self.local_map.a],
        #                            self.local_map.obstacle_map)# [cellsize,center,Q0]
        #diver=self.fe.get_M_update(self.kf.x)                        
        #diver2=self.local_map.get_divergence()
        
        
        for k in range(N):
            #import pdb;pdb.set_trace()
            #0=time.time()
            u=[self.u_controls[0][k],self.u_controls[1][k]]                        
            # make a loop for simulate the robot movement for each control signal
            self.u_control=np.array([[u[0],u[1]]]).T  
            self.fake_sanesor()               
            self.kf.update_matrix(self.matching)   
            self.kf.update(u=self.u_control,z=self.cone_centers.reshape(-1,1))
            #build and publish covariance matrix
            
            #self.local_map.full_map=self.fe.get_M()
            self.build_vector_states()
            self.states_cova.publish(self.states_sended) # Warning! needs to create the topic before
            # Send tf estimated
            self.send_tf_simulator()
            # make a ROS message and then pass data by memory (next lines)
            self.local_map.full_data=self.states_sended
            #time1=time.time()
            self.local_map.run()        
            #print('tiempo requerido:', time.time()-time1)    
            self.local_map.build_map_msg()
            self.map_topic.publish(self.local_map.m)   
            #print('tiempo requerido:', time.time()-t0)
            #t0=time.time()
            #import pdb; pdb.set_trace()
            #self.kf.x[0][0]=self.kf.x[0][0]+self.u_control[0][0]
            #self.kf.x[0][1]=self.kf.x[0][1]+self.u_control[1][0]
            #diver=self.fe.get_M_update(self.kf.x)
            diver2=self.local_map.get_divergence()
            
            #print('Comparacion de divergencias. Diver fast:',diver,' Diver slow:',diver2)
            #import pdb;pdb.set_trace()
            #print('tiempo requerido:', time.time()-t0)
            #import pdb; pdb.set_trace()

            #if self.avoid_collision():
            #    print('Colision detectada')
            #    colission_detected=True
            #    break

            self.rate.sleep()
        return False,[]
        # 
        #print(self.u_controls)
        if colission_detected:
            bias=0.0
            new_u=[self.kf.x[0,0]-x0[0],self.kf.x[0,1]-x0[1]]

            # security range setted by bias variable
            norm=np.math.sqrt(new_u[0]**2+new_u[1]**2)
            if norm>bias:
                norm2=norm-bias
                new_u[0]=new_u[0]*norm2/norm
                new_u[1]=new_u[1]*norm2/norm
            
            self.collision_detected=True
            return True,new_u
        else:
            return False,[]
            
    def avoid_collision(self):
        """ This function check if a obstacle is close to the agent
        """
        x_abs=self.kf.x[0,0]
        y_abs=self.kf.x[0,1]
        r_pose=np.array([[x_abs,y_abs]])
        obstacles=self.local_map.obstacle_map
        cz=self.local_map.cell_size
        mask=obstacles>self.threshold_obstacle # $1 this is arbitrary threshold
        obs=np.where(mask)
        obs=np.array(obs).T
        a=self.local_map.map_size[0]*cz/2
        b=self.local_map.map_size[1]*cz/2
        obs=obs*cz-np.array([[a,b]])
        #import pdb;pdb.set_trace()
        dist=np.linalg.norm(obs-r_pose,axis=1)
        if dist.shape[0]==0:# No hay obstaculos
            return False
                
        mi=np.min(dist)
        
        #if self.last_obst_dist==1000.0:
        #    self.last_obst_dist=mi
        #    return False
        
        # import pdb;pdb.set_trace()
        if mi<self.minimum_distance:# and mi<self.last_obst_dist: # if the distance of obstacle is decreasing
            #import pdb;pdb.set_trace()
            return True
        else:
            return False
    
    def probability_trajectory(self):
        """ This function compute the probability of the trajectory.
        Is a test function
        Esta funcion no se usa. No pude resolver el problema cicloestacinario.
        """
        
        cz=self.local_map.cell_size
        r_pose=(np.array([[self.kf.x[0,0],self.kf.x[0,1]]])/cz)
        a=self.local_map.map_size[0]/2
        b=self.local_map.map_size[1]/2
        r_pose=r_pose+np.array([[a,b]])
        p=self.local_map.full_map[int(r_pose[0,0]),int(r_pose[0,1])]
        p=self.local_map.current_uncertainty
        return p

class MAP_:
    """ Generic implementation of a map
    elements:
    .- map_: map as a list of cells
    .- flag_: flag to know if the map is updated or not
    """
    def __init__(self):
        self.map_=[]
        self.flag_=False

class GET_data_:
    """ This class is for get data from main slam and main map and save it in variables
    
    Warning: THIS CLASS MAKE A NODE IN ROS!!!
    
    This class listen /slam/states_cova topic and /exp_map topic and save this data
    in:
    .- self.states: states vector
    .- self.P: covariance matrix
    .- self.exp_map: exploration map
    
    For this class to work, you need to publish once the topics /slam/states_cova and /exp_map however
    it commente the "if sefl.got_states" and "if self.got_exp_map" in the callback functions you can listen 
    all the time the topics.
    """
    
    def __init__(self):
        self.exp_map=MAP_()
        self.land_map=MAP_()
        self.obstacle_map=MAP_()
        #self.exp_map=[]
        #self.land_map=[]
        #self.got_exp_map=False
        #self.got_land_map=False
        self.got_states=False

        self.data_source='/slam/states_cova'
        self.data_source_topic=rospy.Subscriber(self.data_source,states_,callback=self.states_callback,queue_size=1)
        exploration_map_topic='/exp_map'
        #self.data_source_topic=rospy.Subscriber(exploration_map_topic,states_,callback=self.get_exp_map_callback,queue_size=1)
        self.data_source_topic=rospy.Subscriber(exploration_map_topic,states_,callback=self.get_map_callback,
                                                callback_args=self.exp_map,queue_size=1)
        landmarks_map_topic='/land_map'
        self.data_source_topic=rospy.Subscriber(landmarks_map_topic,states_,callback=self.get_map_callback,
                                                callback_args=self.land_map,queue_size=1)
        obstacle_map_topic='/obstacle_map_matrix'
        self.data_source_topic=rospy.Subscriber(obstacle_map_topic,states_,callback=self.get_map_callback,
                                                callback_args=self.obstacle_map,queue_size=1)
        
        rospy.init_node('simulation_node',anonymous=False)
        return
                
        
    def states_callback(self,data=states_()):
        """ Build states and covariance matrix P from Float32MultiArray topic
        First data is a state vector and the follow one is an covariance matrix
        """
        if self.got_states:
            return # only enter once
        N=data.layout.dim[0].size
        self.P=[]
        self.states=list(data.data[0:N])
        #self.states_0=list(data.data[0:N])
        for i in range(N):
            i_0=(i+1)*N
            i_1=(i+2)*N
            self.P.append(list(data.data[i_0:i_1]))

        self.got_states=True

    def get_data(self):
        """ Return states, covariance matrix and exp_map
        """
        return self.states,self.P,self.exp_map.map_,self.land_map.map_,self.obstacle_map.map_
    
    def get_exp_map_callback(self,data=states_):
        """ recibe from map server the exploration map 
        Get Float32Multiarray message with exploration map.
        This function was checked in test.py for little arrays
        """
        #import pdb;pdb.set_trace()
        if self.got_exp_map:
            return # only enter once
        s=[data.layout.dim[0].size,data.layout.dim[1].size ]
        exp_map=[]
        # TODO: check if is s[0] and then  s[1] or s[1] and then s[0]
        for k in range(s[0]):
            exp_map.append(list(data.data[k*s[1]:(k+1)*s[1]]))
        
        self.exp_map=np.array(exp_map) 
        self.got_exp_map=True
    
    def get_map_callback(self,data,map_data=MAP_()):
        """ recibe from map server some map and save it in output_ variable
        Get Float32Multiarray message.
        This function was checked in test.py for little arrays
        """
        #print('argumento: ',arg)
        #output_=arg['map']
        #flag_=arg['flag']
        if map_data.flag_:
            return # only enter once
        s=[data.layout.dim[0].size,data.layout.dim[1].size ]
        map_=[]
        
        for k in range(s[0]):
            map_.append(list(data.data[k*s[1]:(k+1)*s[1]]))
        
        map_data.map_=np.array(map_) 
        map_data.flag_=True
        #print('argumento: ',arg)

def _convertir_base(numero, base, n_digits):
    conversion = ""
    while numero > 0:
        residuo = numero % base
        conversion = str(residuo) + conversion
        numero = numero // base

    return conversion.zfill(n_digits)

def make_control_signals():
    """ Make a list of control signals
    There are a set of control signals that will be use to generate
    the x_set and y_set and then make a grid of control signals.
    
    For generate all sigals, convinatory theory is used.
    """
    generator_set=rospy.get_param('distance_branch')
    #branch=rospy.get_param('distance_branch')
    #generator_set=[-branch,branch,0] # base $1
    #generator_set=[-2,2,0] # base
    #generator_set=[-20,20,0,4,-4] # base
    depth=2 # x,y
    number_convinations=len(generator_set)**depth
    numbers_set=np.linspace(0,number_convinations-1,number_convinations,dtype=int)
    u_signals=[]
    for num in numbers_set:
        n=_convertir_base(num,len(generator_set),depth)
        l=[]
        for k in range(depth):
            l.append(generator_set[int(n[k])])
        u_signals.append(l)  
    return u_signals 

def make_control_signals_extended():
    u_base=make_control_signals()
    u_total=[]
    for u in u_base:
        for u_ in u_base:
            u_total.append([u, u_])

    return u_total

def make_control_signal_from_path(path,paso=0.3):
    """ Make a list of control signals from RRT path
    el path es una lista de tuplas, es decir [(x1,y1),(x2,y2),...,(xn,yn)]
    return a list of lists of control signals of <paso> length each one
    """
    #paso=0.3
    #import pdb;pdb.set_trace()
    if len(path) < 2:
        raise ValueError("La lista de puntos debe contener al menos dos puntos para la interpolación.")
    # Inicializa las listas para almacenar las coordenadas x e y interpoladas
    x_interp = [];   y_interp = []
    for i in range(1, len(path)):
        x0, y0 = path[i - 1] ;        x1, y1 = path[i]
        # Calcula la distancia entre los puntos
        distance = np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        # Calcula la cantidad de pasos necesarios para alcanzar una distancia de 0.3m
        num_steps = int(distance / paso)+1 # +1 es para que no se pase
        # Calcula los incrementos para x e y
        dx = (x1 - x0) / num_steps;        dy = (y1 - y0) / num_steps
        for j in range(num_steps):
            # Interpola linealmente entre los puntos
            x_interp.append(x0 + j * dx);            y_interp.append(y0 + j * dy)

    # Agrega el último punto al resultado
    x_interp.append(path[-1][0]);    y_interp.append(path[-1][1])
    #import pdb;pdb.set_trace()
    return np.diff(np.array([x_interp, y_interp]).T,axis=0),len(x_interp)*paso

class Simulator_manager:
    """ Catch data from slam and map servers and simulate posible movements
    
    This class depends on Get_data class and SLAM2D_simulated class.
    You can define the step of each movement, and then use a external function
    called make_control_signals to generate a list of control signals in a
    particular way. 
    The principal method is instances_slam2d_simulated that will generate a
    control action for each posible control signal and then make an instance
    of SLAM2D_simulated class. Each instance is a simulation of movements and
    get the divergence of  full map.
    
    TODO: in instances_slam2d_simulated method, do each one with a thread.
    Another posible way is make this class in golang and use goroutines.
    Maybe is better to use c++. For this  version each simulation in
    sim_.run() expend 1 second aprox.
    
    """
    def __init__(self,debug=0,ros_required=True,minimum_distance=0.5):
        """if debug==1 print divergence, if debug==2 publish a map and take its time..."""
        self.step_halucination=0.3 # ver calculo realizado en simulated Slam
        self.step=self.step_halucination # step of the robot
        self.minimun_distance=minimum_distance
        self.debug=debug
        if ros_required:
            self.with_ros()
        else:
            self.without_ros()
            # hay que cargar las variables states, P, exp_map, obstacle_map, path
            #luego ejecutar self.run()
            pass        
       
    def run(self):
        self.u_signals,distance=make_control_signal_from_path(self.path,self.step_halucination)
        diver=self.instances_slam2d_simulated2() # no sale de esta funcion a veces...
        #print('Salgo 3')
        return diver,distance
    
    def without_ros(self):
        self.states=0
        self.P=0
        self.exp_map=0
        self.obstacle_map=0
        self.path=0


    def with_ros(self):
        rospy.init_node('simulation_node',anonymous=True)
        self.gazebo= GazeboSimulationController()
        self.gazebo.unpause_simulation()
        admin=GET_data()
        while not (admin.got_states and admin.exp_map.flag_ and  admin.obstacle_map.flag_): # wait for a states data
            print('waiting for data')
            print('states: ',admin.got_states, 'exp_map: ',admin.exp_map.flag_,'obstacle_map: ',admin.obstacle_map.flag_)            
            time.sleep(1)

        rospy.loginfo('data captured')
        self.states,self.P,self.exp_map,self.obstacle_map=admin.get_data()      
        if rospy.get_param('UF'):
            pass

        publisher_=rospy.Subscriber('/path_rrt_star',Path,self.path_callback,queue_size=1)
        self.new_path=False
        print('waiting for path')
        while not self.new_path:            
            time.sleep(1)
        
        self.run()

    def path_callback(self,data=Path):
        print('new path received')
        self.new_path=True
        self.path=[]
        for p in data.poses:
            self.path.append((p.pose.position.x,p.pose.position.y))
    
    def make_u_controls_old(self,u_signal):
        """convert from objetive position to control signals

        u_signal=[x,y]: objetive position
        """
        x=u_signal[0];y=u_signal[1]
        x_steps=int(x/self.step);y_steps=int(y/self.step) # amount of steps in each direction
        # if one of steps is zero then will be repeat zero step for each step of the other
        mag_x=1;mag_y=1
        if abs(x_steps)>abs(y_steps):
            if y_steps==0:
                y_steps=abs(x_steps)
                mag_y=0
            else:
                y_steps=int(y_steps*abs(x_steps)/abs(y_steps))
        elif abs(x_steps)<abs(y_steps):
            if x_steps==0:
                x_steps=abs(y_steps)
                mag_x=0
            else:
                x_steps=int(x_steps*abs(y_steps)/abs(x_steps))
        elif x_steps==0 and y_steps==0:
            x_steps=1;y_steps=1
            
        return [[self.step*np.sign(x)*mag_x]*abs(x_steps),[self.step*np.sign(y)*mag_y]*abs(y_steps)]
    
    def make_u_controls(self,u_signal):
        """convert from objetive position to control signals

        u_signal=[x,y]: objetive position
        """
        #import pdb;pdb.set_trace()
        x=u_signal[0];y=u_signal[1]
        x_steps=int(x/self.step);y_steps=int(y/self.step) # amount of steps in each direction
        # if one of steps is zero then will be repeat zero step for each step of the other
        n=max(abs(x_steps),abs(y_steps))
        if n==0:
            v=[[0],[0]]
        else:
            v=[[(x/n)]*n,[(y/n)]*n]
        return v        
    
    def instances_slam2d_simulated(self):
        """ Take the control signals and create a class SLAM2D_simulated instances for get gain information
        """
        #TODO: make a thread for each instance
        #TODO: make a instance of map manager
        max_diver=-1000
        best_u=[[0,0],[0,0]]
        for u in self.u_signals:
            
            sim_=SLAM2D_simulated(debug=True)# also publish a map and its states
            sim_.P=copy(self.P)
            sim_.states=copy(self.states )
            sim_.local_map.exploration_map=copy(self.exp_map)
            #sim_.local_map.landmarks_map=copy(self.land_map)
            sim_.local_map.obstacle_map=copy(self.obstacle_map)
            #print('signals sended: ',u_control)            
            #u_control=self.make_u_controls_extended(u)
            u_control=self.make_u_controls(u)
            sim_.u_controls=copy(u_control)

            # Primer paso
            if self.debug==2:   
                flag,u_new=sim_._run_debug()
                if flag: # an obstacle was detected
                    u=u_new
            else:
                flag,u_new=sim_.run()
                if flag: # an obstacle was detected
                    u=u_new
            # Segundo Paso
            for u_ in self.u_signals:
                sim_2=self.full_copy(sim_)
                u_control=self.make_u_controls(u_)
                sim_2.u_controls=copy(u_control)
                if self.debug==2:   
                    flag,u_new=sim_2._run_debug()
                    if flag: # an obstacle was detected
                        u_=u_new
                else:
                    flag,u_new=sim_2.run()
                    if flag: # an obstacle was detected
                        u_=u_new

                rnd=5*np.random.randn()*0 # ojo se desactiva el ruido 
                current_diver=sim_2.local_map.get_divergence()+rnd
                if self.debug!=0:
                    print('control signals: ',[u,u_])
                    print("divergencia: ",current_diver,'random: ',rnd)

                del(sim_2)
            #import pdb;pdb.set_trace()
                if current_diver>max_diver: #and (np.math.sqrt(u[0]**2+u[1]**2)>1.0 or (u[0]==0 and u[1]==0)): # los movimientos tienen uqe ser mayores a 1.0m para ser considerados, salvo que sea un movimiento nulo
                    max_diver=current_diver
                    best_u=[u,u_]

        #print('best control signals: ',best_u, 'best divergencia: ',max_diver)
        print(best_u)
        self.best_u=best_u
        self.max_diver=max_diver
        return best_u,max_diver

    def full_copy(self,sim_,parameters):
        sim_2=SLAM2D_simulated(debug=True,parameters=parameters)# alse publish a map and its states
        sim_2.P=copy(sim_.P)
        sim_2.states=copy(sim_.states)
        sim_2.local_map.exploration_map=copy(sim_.local_map.exploration_map)
        #sim_2.local_map.landmarks_map=copy(sim_.local_map.landmarks_map)
        sim_2.local_map.obstacle_map=copy(sim_.local_map.obstacle_map)
        sim_2.kf=copy(sim_.kf)     
        sim_2.continue_simulation=True
        #import pdb;pdb.set_trace()   
        return sim_2
    
    def full_copy_fe(self,sim_,parameters):
        sim_2=self.full_copy(sim_,parameters)
        
        sim_2.fe=copy(sim_.fe)
        #sim_2.fe.X=copy(sim_.fe.X)
        #sim_2.fe.FOV=copy(sim_.fe.FOV)
        #sim_2.fe.parameters=copy(sim_.fe.parameters)
        #sim_2.fe.k=copy(sim_.fe.k)
        #sim_2.fe.V0=copy(sim_.fe.V0)
        #sim_2.fe.M=copy(sim_.fe.M)
        #sim_2.fe.k=copy(sim_.fe.k)
        #sim_2.fe.k=copy(sim_.fe.k)
        return sim_2
    
    def make_u_controls_extended(self,u_signal):
        """
        Calls make_u_controls for each u_signal and return a list of u_controls
        """
        #import pdb;pdb.set_trace()
        u_total=[[],[]]
        for u in u_signal:
            u_control=self.make_u_controls(u)
            u_total[0].extend(u_control[0])
            u_total[1].extend(u_control[1])
        
        return u_total
    def instances_slam2d_simulated2(self)->float:
        """
        Este es la funcion que administra las simulaciones
        $1 $2
        """
        # TODO: make a thread for each instance
        # TODO: make a instance of map manager
        max_diver = 0
        #pool = Pool(cpu_count()-2)

        results = []
        P0=rospy.get_param('P_0')
        R=rospy.get_param('R_noise')
        Q=rospy.get_param('Q_noise')
        new_landmark=rospy.get_param('max_distance_landmark')
        sigma_max=rospy.get_param('sigma_max')
        fov=rospy.get_param('fov')
        cell_size=rospy.get_param('cell_size')
        self.cell_size=cell_size
        parameters = [P0,R,Q,new_landmark,sigma_max,fov,cell_size,self.minimun_distance] #P0, R, Q, new_landmark,sigma_max,fov,cell_size,minimun distance
        temp=rospy.get_param('planner')
        diver=self.run_sim_simple(self.u_signals, parameters)
        #print("salgo 2")
        #import pdb; pdb.set_trace()
        return diver



        #import pdb;pdb.set_trace()
        if temp=='double':
            simulation=2
        if temp=='simple':
            simulation=1
        
        try:
            print('simulation ', simulation)
        except:
            print('No fue posible imprimir el valor del tipo de simulacion')
            import pdb;pdb.set_trace()
            
        timeout_=30 
        
        #u_extra=self.get_aditional_u_control()
        #import pdb;pdb.set_trace()
        #self.make_u_controls(u_extra)
        u_extra,flag=self.get_aditional_u_control() # No se que hace esto
        if not flag:
            print('Proceso finalizado por falta de fronteras')
            print(-1.0) # divergencia
            print([[0,0],[0,0]])
            return 

        if rospy.get_param('UF'):
            #u_extra=self.get_aditional_u_control()
            self.u_signals_extra=self.u_signals+[u_extra]
            #self.marker.set_marker_position(u_extra[0],u_extra[1],0)
            #self.marker.marker.header.stamp = rospy.Time.now()
            #self.marker.marker_pub.publish(self.marker.marker)
        else:
            self.u_signals_extra=self.u_signals
            
        if not self.debug==2:
            self.gazebo.pause_simulation()
        #u_extra=[u_extra]+[u_extra]
        #print(u_extra)
        #self.u_signals_extra=self.u_signals#+[u_extra]
        
        #for u in self.u_signals:
        for u in self.u_signals_extra:
            #print(u,self.make_u_controls3(u))
            if simulation==1 and self.debug==2:
                results.append(list(run_sim_simple(u, self,parameters)))
            elif simulation==2 and self.debug==2:
                results.append(list(run_sim_double(u, self,parameters)))
            elif simulation==1:
                results.append(pool.apply_async(run_sim_simple, (u, self,parameters)))

            if simulation==2:
                results.append(pool.apply_async(run_sim_double, (u, self,parameters))) 
            
                
            

            #    results.append(pool.apply_async(run_sim_double, (u, self)))
            #if rospy.get_param('planner')=='simple':
            #    results.append(pool.apply_async(run_sim_simple, (u, self)))
            
            
            # results.append(pool.apply_async(run_sim_double, (u, self))) #$1
            #results.append(pool.apply_async(run_sim_simple, (u, self,parameters)))
        # for result in results:
        #  try:
        #      result.get(timeout=timeout_)
        #  except TimeoutError:
        #      # Aquí puedes manejar el caso en el que la tarea supere el timeout
        #      print("La tarea excedió el tiempo máximo de espera.")
        #      result.terminate()
        if not self.debug==2:
            pool.close()
            pool.join()
              
      

        best_u = None
        max_diver = -1000
        list_diver=[]
        list_u=[]                

        for result in results:
            if self.debug==2:
                u, diver = result
            else:
                u, diver = result.get()

            list_diver.append(diver)
            list_u.append(u)


            if diver > max_diver:
                max_diver = diver
                best_u = u
            #u, diver = result.get() # $1 activar esto
            #if abs(diver - max_diver)<1.0:
            #    rnd=5*np.random.randn()
            #    diver_=diver+rnd
            #    #print('Se agrega ruido a la divergencia: ',rnd)
            #    if diver_ > max_diver:
            #        max_diver = diver_
            #        best_u = u
            #        print('ganancia seleccionada: ',best_u,'random: ',rnd)
            #else:
            #    if diver > max_diver:
            #        max_diver = diver
            #        best_u = u
            #        #print('ganancia seleccionada: ',best_u)
        #max_diver,best_u=divergence_analysis(max_diver,best_u,list_diver,list_u)
        #umbral=1.0
        #if abs(best_u[0][0])<umbral and abs(best_u[0][1])<umbral and abs(best_u[1][0])<umbral and abs(best_u[1][1])<umbral:
        #    best_u=[[0.0,0.0],[0.0,0.0]]     # si le pide moverse poco, entonces da la condicion de stop   
        
        #list_diver=self.match_closest_path(list_u,list_diver)
        #for i in range(len(list_diver)):
        #    if list_diver[i]>max_diver:
        #        max_diver=list_diver[i]
        #        best_u=list_u[i]
        #import pdb;pdb.set_trace()
        if not self.debug==2:
            self.gazebo.unpause_simulation()

        print(max_diver)
        print(best_u)
        self.best_u = best_u
        self.max_diver = max_diver
        # Cuidado: list_diver y list_u no contienen todas las divergencias y las acciones de control. Solo tienen el maximo de cada hilo de ejecucion...
    def run_sim_simple(self,u, parameters):
        sim_ = SLAM2D_simulated(debug=True,parameters=parameters)
        sim_.P = copy(self.P)
        sim_.states = copy(self.states)
        sim_.local_map.exploration_map = copy(self.exp_map)
        sim_.local_map.obstacle_map = copy(self.obstacle_map)
        sim_.u_controls = copy(u.T)
        max_diver = -1000
        max_diver2=-1000
        if self.debug == 2:
            flag, u_new = sim_._run_debug()
            if flag:  # an obstacle was detected
                u = u_new
        else:
            flag, u_new = sim_.run()
            if flag:  # an obstacle was detected
                u = u_new

        rnd = 5 * np.random.randn() * 0  # ojo se desactiva el ruido
        diver_common=sim_.local_map.get_divergence()
        #print('Divergencia obtenidaaaa: ',diver_common)
        return diver_common
    
    def get_aditional_u_control(self):
        
        p_objetive,D,distance,flag=self.get_frontiers()
        if flag==False:
            return [0,0],flag

        p_objetive=(p_objetive-300)*self.cell_size # $1 300 es el centro del mapa, aca tiene que ir una funcion de las dimenciones del mapa
        #import pdb;pdb.set_trace()
        return [p_objetive[0]-self.states[0],p_objetive[1]-self.states[1]],flag
                        
    def match_closest_path(self,list_u,list_diver):
        p_objetive,D,distance,flag=self.get_frontiers()
        if flag==False:
            return list_diver
        p_objetive=p_objetive[:, np.newaxis]
        #p_objetive=np.array([[p_objetive[1,0],p_objetive[0,0]]])
        n,m=self.exp_map.shape
        cx=n/2;cy=m/2
        pose=self.states[:2]        
        poses_fin = np.empty((0, 2))
        #poses_fin=np.array([[]])
        for u in list_u:
            pose_fin=np.array([[pose[0]+u[0][0]+u[1][0],pose[1]+u[0][1]+u[1][1]]])/self.cell_size+np.array([[cx,cy]])
            poses_fin=np.vstack([poses_fin, pose_fin])
            
        #import pdb;pdb.set_trace()  
        dist=np.power(np.power((poses_fin-p_objetive.T),2).sum(axis=1),0.5)
        for i in range(len(dist)):
            list_diver[i]=list_diver[i]+D/dist[i]
            
        #idx=np.argmin(dist)
        #list_diver[idx]=list_diver[idx]+D/distance  # $1 5 es la distancia minima, aca tiene que ir una funcion del fov
        return list_diver

    def get_frontiers(self):
        uf=UFrontier(beta=rospy.get_param('beta'))
        M=self.exp_map+self.land_map-np.multiply(self.exp_map,self.land_map)
        uf.set_maps(M=M,obstacle_map=self.obstacle_map)
        pose_=np.array([self.states[0],self.states[1]])
        p_objetive,distance,flag=uf.find_frontiers(pose=pose_)
        D=0
        if flag:
            D=uf.get_aprior_divergence(p_objetive)
        
        return p_objetive,D,distance,flag        

def divergence_analysis(max_diver,best_u,divers,us):
    """
    Selecciona de forma aleatoria entre los que tiene divergencia maxima
    """
    idx=[]
    for i in range(len(divers)):
        if abs(divers[i] - max_diver)<2.0:            
            idx.append(i)

    if len(idx)>1:
        idx_=np.random.choice(idx)
        while us[idx_]==[[0,0],[0,0]]:
            idx_=np.random.choice(idx)
        max_diver=divers[idx_]
        best_u=us[idx_]
        print('*seleccionado aleatoriamente')
    return max_diver,best_u

def run_sim_double(u, self,parameters):
    sim_ = SLAM2D_simulated(debug=True,parameters=parameters)
    sim_.P = copy(self.P)
    sim_.states = copy(self.states)
    sim_.local_map.exploration_map = copy(self.exp_map)
    #sim_.local_map.landmarks_map = copy(self.land_map)
    sim_.local_map.obstacle_map = copy(self.obstacle_map)

    u_control = self.make_u_controls(u)
    sim_.u_controls = copy(u_control)
    max_diver = -1000
    if self.debug == 2:
        flag, u_new = sim_._run_debug()
        if flag:  # an obstacle was detected
            u = u_new
    else:
        flag, u_new = sim_.run()
        if flag:  # an obstacle was detected
            u = u_new

    for u_ in self.u_signals_extra:
        sim_2 = self.full_copy_fe(sim_,parameters)
        u_control = self.make_u_controls(u_)
        sim_2.u_controls = copy(u_control)

        if self.debug == 2:
            flag, u_new = sim_2._run_debug()
            if flag:  # an obstacle was detected
                u_ = u_new
        else:
            flag, u_new = sim_2.run()
            if flag:  # an obstacle was detected
                u_ = u_new

        rnd = 5 * np.random.randn() * 0  # ojo se desactiva el ruido
        #current_diver = sim_2.local_map.get_divergence() + rnd
        current_diver =sim_2.fe.get_divergence()

        #p=sim_.traj_prob[0]/sim_.traj_prob[1]
        #current_diver=(current_diver-sim_.initial_divergence)*p
        #print('Divergencia inicial: ', sim_.initial_divergence)
        #print('Probabilidad de la trayectoria: ',p)

        if self.debug != 0:
            print('control signals: ', [u, u_])
            print("divergencia: ", current_diver, 'random: ', rnd)

        del sim_2

        if current_diver > max_diver:  # and (np.math.sqrt(u[0]**2+u[1]**2)>1.0 or (u[0]==0 and u[1]==0)):
            max_diver = current_diver
            best_u = [u, u_]

    return best_u, max_diver

def run_sim_simple(u, self,parameters):
    sim_ = SLAM2D_simulated(debug=True,parameters=parameters)
    sim_.P = copy(self.P)
    sim_.states = copy(self.states)
    sim_.local_map.exploration_map = copy(self.exp_map)
    #sim_.local_map.landmarks_map = copy(self.land_map)
    sim_.local_map.obstacle_map = copy(self.obstacle_map)

    #u_control = self.make_u_controls(u)
    #sim_.u_controls = copy(u_control)
    sim_.u_controls = copy(u.T)

    
    max_diver = -1000
    max_diver2=-1000
    if self.debug == 2:
        flag, u_new = sim_._run_debug()
        if flag:  # an obstacle was detected
            u = u_new
    else:
        flag, u_new = sim_.run()
        if flag:  # an obstacle was detected
            u = u_new

   

    rnd = 5 * np.random.randn() * 0  # ojo se desactiva el ruido
    #current_diver = sim_.local_map.get_divergence() + rnd
    diver_common=sim_.local_map.get_divergence()
    #diver_fast=sim_.fe.get_divergence()
    #current_diver = sim_.accumulated_diver + rnd
    #p=sim_.traj_prob[0]/sim_.traj_prob[1]
    #current_diver=(current_diver-sim_.initial_divergence)*p
    #print('Divergencia inicial: ', sim_.initial_divergence)
    #print('Probabilidad de la trayectoria: ',p)
    print('Divergencia obtenida: ',diver_common)
    return diver_common
    if self.debug != 0:
        print('control signals: ', [u, [0,0]])
        #print("divergencia: ",  sim_.local_map.get_divergence(), 'random: ', rnd,'accumulated divergence: ',sim_.accumulated_diver)
        #print("divergencia: ", current_diver, 'random: ', rnd)
        print("divergencia fast: ", diver_fast)
    del sim_
    current_diver=diver_fast
    if current_diver > max_diver:  # and (np.math.sqrt(u[0]**2+u[1]**2)>1.0 or (u[0]==0 and u[1]==0)):
        max_diver = current_diver
        best_u = [u, [0,0]]        

    return best_u, max_diver

def _main_debug():
    sim=Simulator_manager()
    return sim.best_u

def main(data=-1):
    if data!=-1:
            sim=Simulator_manager(data)
    else:
            sim=Simulator_manager()

    rospy.signal_shutdown("Proceso finalizado")
    return 

    
if __name__=='__main__':
    if len(sys.argv)>1:
        main(int(sys.argv[1]))
    else:
        main()
        
          

        
    


   # a=_main_debug()
    

    #_main_debug()
    
    # git add simulated_2d_slam.py
    # git commit -m "casi que no se hicieron cambios en el codigo. Solo se agregaron comentarios y los headers de cada clase. Faltaria ver que
    # mensajes hay que enviar a modo de informacion o de debug sumado a los test unitarios."
    

  
    
    