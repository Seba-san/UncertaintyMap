#!/usr/bin/env python3
"""
This code get position of robot and landmarks and, on the other hand get the covariance data for make this maps:
- Landmark map
- Exploration map
"""

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray as states_
from std_msgs.msg import MultiArrayDimension 
from std_msgs.msg import Float32 as divergence_msg
from scipy.stats import multivariate_normal as norm_distribution #https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.multivariate_normal.html
import scipy
from nav_msgs.msg import OccupancyGrid as map_msg
from copy import deepcopy as copy
import time
import tf2_ros
import geometry_msgs.msg
import sys
from sensor_msgs.msg import LaserScan as lasermsg

from numpy.linalg import eig

from raycasting import RayCasting
from several_classes import Orientation
from scipy.signal import convolve2d

from scipy.io import loadmat
#import os

import matplotlib.pyplot as plt

class MapServer:
    def __init__(self,states_cova_topic='/slam/states_cova',
                 center_frame='robot1_tf/odom_groundtruth',
                 topic_map='/slam/landmark_map',
                 exp_map_topic='/exp_map', debug=False,
                 sb=0.6,fov=5,cell_size=0.1,bbox=[],map_size_m=[60,60]):
        
        self.debug=debug
        if debug:            
            self.sub=rospy.Subscriber(states_cova_topic,states_,self.get_states,queue_size=1)            
            self.map_topic=rospy.Publisher(topic_map,map_msg,queue_size=1)
            self.publish_divergence=rospy.Publisher('/divergence',divergence_msg,queue_size=1)
            self.exploration_map_publisher=rospy.Publisher(exp_map_topic,states_,queue_size=1)
            #self.landmarks_map_publisher=rospy.Publisher('/land_map',states_,queue_size=1)
            # begin laser process 
            laser_topic='/robot1/laser/scan_lidar_horizontal'
            self.obstacle_map_sub=rospy.Subscriber(laser_topic,lasermsg,self.laser_callback,queue_size=1)
            self.laser_angles=np.linspace(-np.pi,np.pi,int(1+np.pi*2/0.008738775737583637))
            self.obstacle_map_topic=rospy.Publisher('/obstacle_map',map_msg,queue_size=1)
            self.obstacle_map_topic_matrix=rospy.Publisher('/obstacle_map_matrix',states_,queue_size=1)
            self.ori=Orientation()
            
            # end laser process
            
        
        #self.cell_size=0.1# square cells
        self.cell_size=cell_size #rospy.get_param('cell_size')
        #sb=rospy.get_param('sigma_max')
        #sb=0.6 # max sigma, is the sigma asociated to beta
        #self.FOV=5#m circular
        self.FOV=fov# es una distancia rospy.get_param('fov')

        # Variables
        self.sigma_max2=sb**2
        A=np.array([[self.sigma_max2, 0],[0,self.sigma_max2]]) 
        self.beta=self.get_prob_from_cov(A)
        self.a=np.power(self.beta,0.5)*sb
        
        # Esto que sigue es para 1D no da el mismo valor para 2d
        #b=-0.717;a=-0.416; # gaussian cdf aproximation
        #x=self.cell_size/(2*sb)
        #beta_=1-np.exp(b*x+a*x**2)
        #self.alpha=2*self.beta-self.beta**2
        self.full_data=0 # is for set states and covariace without ROS

        #self.alpha=0.005/2#0.001/2#0.0001 # Tendria qeue llamarse Beta este parametro
        if rospy.get_param('previous_map_flag'):
            self.previous_map()
        else:
            self.build_maps(bbox=bbox,map_size_m=map_size_m)
        
        self.new_data=False                
        self.states=np.array([0,0])
        self.make_mask_FOV()
        self.new_data_=False # for the first callback of laser. It avoids an erroneous first update of the map
        self.new_laser_data=False        
        self.exploration_flag=False #when is true, make a exploration task without max divergence
        
        self.center_frame=center_frame
        self.RC=RayCasting()
        self.RC.cell_size=self.cell_size
        self.RC.FOV=self.FOV
        self.insert_pose=False # for insert pose in landmarks map
        self.K_update=0.5#0.5

        # Uncertainties
        self.stita=0.125*np.pi/180;self.sr=0.12;self.sfi=0.125*np.pi/180

        self.current_uncertainty=self.beta
        
        # For planification propuses
        self.create_uncertainty_fov()
    
    def previous_map(self):
        #current_directory = os.getcwd()
        #print("###################!!!"+current_directory)
        # Nombre del archivo .mat a leer
        file_name = 'previous_map.mat'        
        # Carga el archivo .mat
        data = loadmat('./'+file_name)
        
        # Obtén las variables 'obstacle_map' y 'exp_map' del archivo
        self.obstacle_map = data['obstacle_map']
        self.exploration_map= data['exp_map']
        self.map_size=self.obstacle_map.shape
        self.exploration_map_bool=np.full(shape=self.map_size,fill_value=False)
        

    def build_maps(self,bbox=[],map_size_m=[60,60]):
        """build all maps if there aren't previous maps

        Args:
            bbox (list, optional): _description_. Defaults to [].
            map_size_m (list, optional): _description_. Defaults to [60,60].
        """
        self.map_size=[int(map_size_m[0]/self.cell_size),int(map_size_m[1]/self.cell_size)]# 60m x 60m
       
        #self.landmarks_map=np.full(shape=self.map_size,fill_value=self.beta)
        #self.full_map=np.full(shape=self.map_size,fill_value=self.beta*2)

        self.exploration_map=np.full(shape=self.map_size,fill_value=self.beta)
        self.exploration_map_bool=np.full(shape=self.map_size,fill_value=False)
        self.obstacle_map=np.full(shape=self.map_size,fill_value=0.5)
        
        # know parts:
        for box in bbox:
            a=int(box[0][0]/self.cell_size)
            b=int(box[0][1]/self.cell_size)
            c=int(box[1][0]/self.cell_size)
            d=int(box[1][1]/self.cell_size)
            sigma2=box[2]**2
            p_sigma=self.get_prob_from_cov(np.array([[sigma2, 0],[0,sigma2]]))
            self.exploration_map[a:c,b:d]=p_sigma
            self.obstacle_map[a:c,b:d]=box[3]
        
        # TODO: Para evitar que se generen fronteras artificiales hay que hacer una "interfaz suave"
        if len(bbox)>0:
            n=30  
            filtro = np.ones((n, n)) / n**2  # Kernel 10x10 de promedio
            # Aplicar convolución 2D
            self.exploration_map = convolve2d(self.exploration_map, filtro, mode='same', boundary='fill', fillvalue=self.beta)

   
    
    def laser_callback(self,data=lasermsg):
        """ add obstacles to obstacles_map from laser data and robot pose

        Do the same things that exploration map

        Input:
            .- data: laser data
            .- self.states: robot pose
            .- self.orientation: true robot orientation
        Output:
            .- self.obstacle_map: map with obstacles
        """

        if self.new_laser_data:# or not self.new_data:
            return # only flux control, update when the slam data is available
        if not self.new_data_:
            return
        self.new_laser_data=True
        self.RC.data=data 
        
        #import pdb; pdb.set_trace()
        v_data=np.array(list(data.intensities))# Vector data
        mask_=v_data>0.0 # umbral arbitrario
        ranges=np.array(list(data.ranges))
        ranges=ranges[mask_]
        x_rel=ranges*np.cos(self.laser_angles[mask_])
        y_rel=ranges*np.sin(self.laser_angles[mask_])

        obstacles=np.array([x_rel,y_rel]).T
        cc=np.math.cos(self.ori.orientation)
        ss=np.math.sin(self.ori.orientation)
        rotation_matrix=np.array([[cc,-ss],[ss,cc]],dtype='f').T      
        obstacles_rotated=np.matmul( obstacles , rotation_matrix)#+pose_abs # from laser frame to world frame 
        
        x_abs=self.states[0]#x_rel+self.states[0]
        y_abs=self.states[1]#y_rel+self.states[1]
        px=int(x_abs/self.cell_size+self.map_size[0]/2)#; px=px.astype(int)
        py=int(y_abs/self.cell_size+self.map_size[1]/2)#; py=py.astype(int)
        #r=self.FOV
        cz=self.cell_size
        (a,b)=self.exploration_FOV.shape
        fov=np.full(shape=[a,b],fill_value=self.beta)

        base=self.beta/self.current_uncertainty

        try:

            fov[self.exploration_FOV]=0.1 # aca hay que ponderar la incertidumbre (0.1p+(1-p)0.5)   #* (1-base)  + base*0.5 # ver que valor le pone octomap. Espacio vacio 0.1
            fov[((obstacles_rotated[:,0])/cz+a/2).astype(int),((obstacles_rotated[:,1])/cz+b/2).astype(int)]=0.9#*(1-base) +base*0.5 # ver que valor le pone octomap. Espacio ocupado 0.7
        except:
            import pdb; pdb.set_trace()
        #fov=np.matmul( fov , rotation_matrix)

        l_new=self.prob2logodds(fov)
                
        low_x=int(px-a/2);low_y=int(py-b/2)
        uncertainty_region=self.exploration_map[low_x:low_x+a,low_y:low_y+b]
        old_map=self.obstacle_map[low_x:low_x+a,low_y:low_y+b]
        
        uncertanty_idx=uncertainty_region>self.current_uncertainty # lo que se queda igual
        #import pdb; pdb.set_trace()

        l_old=self.prob2logodds(old_map)

        #l_new[uncertanty_idx]=l_old[uncertanty_idx] # Solo se actualiza lo que tiene menor incertidumbre, el resto se sobreescribe

        #l=self.bayesian_update_exploration(lo_new=l_new,lo_old=l_old)
        l=self.bayesian_update(l_new,l_old,k=self.current_uncertainty)
        p=self.logodds2prob(l)

        #mask=self.exploration_FOV
        self.RC.orientation=self.ori.orientation
        self.RC.get_raycasting_mask3()
        #self.RC.scan_offline()
        self.mask=copy(self.RC.mask_raycasting) # solo actualiza lo que puede ver el laser
        aux=copy(self.exploration_map_bool)
        aux[low_x:low_x+a,low_y:low_y+b]=self.mask
        self.obstacle_map[aux]=p[self.mask]# El truco de usar mascaras es disminuir el costo computacional.
        #self.obstacle_map[low_x:low_x+a,low_y:low_y+b]=p
        
    #def update_obstale_map(self):


    def make_mask_FOV(self):
        """ make a mask of field of view to generate map of exploration
        """
        r=self.FOV
        cz=self.cell_size
        a=int(2*r/cz); b=a
        self.exploration_FOV=np.full(shape=[a,b],fill_value=False)
       
        for i in range(a):
            for k in range(b):
                x=float(i)*cz
                y=float(k)*cz
                if (x-r)**2+(y-r)**2<r**2:
                    self.exploration_FOV[i,k]=True
                
        #import pdb; pdb.set_trace()    
    def get_states(self,data=states_()):
        """
        Build states and covariance matrix P from Float32MultiArray topic
        First data is a state vector and the follow one is an covariance matrix
        """
        if not self.debug: # debug=True en el programa principal. TODO: hay que cambiar esta bandera xq no dice lo que realmente es
            data=self.full_data
            
        N=data.layout.dim[0].size
        self.P=[]
        self.states=list(data.data[0:N])
        for i in range(N):
            i_0=(i+1)*N
            i_1=(i+2)*N
            self.P.append(list(data.data[i_0:i_1]))

        self.new_data=True
        self.new_data_=True
        self.new_laser_data=False # control de flujo

    def logodds2prob(self,lo):
        """
        convert from log odds to probability
        lo: log odds
        """

        prob=1-1/(1+np.exp(lo)) # probability 
        return prob
    
    def prob2logodds(self,prob):
        """
        convert from probability to log odds
        prob: probability
        """

        lo=np.log(prob/(1-prob))
        return lo

    def bayesian_update(self,lo_new,lo_old,k=None):
        """
        Apply a bayesian update between a part of old map and a part of newest map
        This is a contribution to the project's work
        lo_new: log odds new
        lo_old: log odds old
        """
        if k is None:
            k=self.K_update
        lo_k=lo_old+k*(lo_new-lo_old)
        return lo_k
    
    def bayesian_update_exploration(self,lo_new,lo_old):
        """ Update the map using a new way. 

        Args:
            lo_new (_type_): _description_
            lo_old (_type_): _description_
        """
      
        lo_kp=self.bayesian_update(lo_new,lo_old)
        # This trick is the same of conditional_update method, but faster
        l_beta=np.log(self.beta/(1-self.beta))
        idx_1=lo_old>=lo_kp
        idx_2=lo_kp>=l_beta
        idx_3=lo_old>l_beta
        idx_4=l_beta>lo_kp
        idx_12=np.logical_and(idx_1,idx_2)
        idx_34=np.logical_and(idx_3,idx_4)
        idx=np.logical_or(idx_12,idx_34)
        lo_kp[idx]=lo_old[idx]
         
        return lo_kp
            
    def get_divergence_e_map(self):
        E=self.exploration_map
        N=2;beta=self.beta
    
        Dkl=(self.cell_size**2)*(np.log(E / beta) - N / 2 + (N / 2) * (beta / E) ** (2 / N))
        idx=E<beta
        Dkl[idx]=-Dkl[idx]
        Dkl_tot=np.sum(Dkl)
        return Dkl_tot
    def get_divergence_landmarks_(self,i):
        """
        a partir de un indice de landmark, calcula la divergencia con signo de ese landmark
        """        
        N=2
        idx_lo=2*i
        idx_up=2*i+1
        sxx=self.P[idx_lo][idx_lo]
        syy=self.P[idx_up][idx_up]
        sxy=self.P[idx_lo][idx_up]
        cov_=np.array([[sxx, sxy],[sxy,syy]])
       
        beta_mat=np.array([[self.sigma_max2, 0],[0,self.sigma_max2]])
        beta_mat_inv=np.array([[1/self.sigma_max2, 0],[0,1/self.sigma_max2]])
        rel=np.linalg.det(beta_mat)/np.linalg.det(cov_)
        Dkl=0.5*(np.log(rel)+np.trace(beta_mat_inv*cov_) -N)
        if rel<1:
            Dkl=-Dkl
        return Dkl
    
    def get_divergence_agent(self):
        # Calcula la divergencia del vehiculo
        N=2
        idx_lo=0
        idx_up=1
        sxx=self.P[idx_lo][idx_lo]
        syy=self.P[idx_up][idx_up]
        sxy=self.P[idx_lo][idx_up]
        cov_=np.array([[sxx, sxy],[sxy,syy]])
       
        beta_mat=np.array([[self.sigma_max2, 0],[0,self.sigma_max2]])
        beta_mat_inv=np.array([[1/self.sigma_max2, 0],[0,1/self.sigma_max2]])
        rel=np.linalg.det(beta_mat)/np.linalg.det(cov_)
        Dkl=0.5*(np.log(rel)+np.trace(beta_mat_inv*cov_) -N)
        if rel<1:
            Dkl=-Dkl
        return Dkl

    def get_divergence(self):
        """
        Calcula la divergencia del mapa, de los landmarks y la del agente
        """
        N=len(self.states)/2-1
        Dkl=0
        for i in range(int(N)):
            Dkl=self.get_divergence_landmarks_(i+1)+Dkl
        
        Dkl=self.get_divergence_e_map()+Dkl+self.get_divergence_agent()
        return Dkl

    def get_prob_from_cov(self,cov):
        """
        Retrieve the area below of 2D normal distribution with parameters of pose
        """
        sxx=cov[0][0]
        syy=cov[1][1]
        sxy=cov[0][1]
        A=np.array([[sxx, sxy],[sxy,syy]]) 
        cs2=self.cell_size/2
        limits_=np.array([cs2,cs2])
        lower_limit=np.array([-cs2, -cs2])        
        return norm_distribution.cdf(x=limits_,cov=A,lower_limit=lower_limit)# check this operation.

    def Sigma_measure(self,cov_a,tita,r,fi):
        '''
        Calcula la matriz de covarianza de la medicion sin tener en cuenta la incertidumbre del agente
        $4 En proceso
        '''
        fitita=fi+tita
        sin=np.math.sin(fitita);cos=np.math.cos(fitita)
        J=np.array([[1,0,-r*sin,cos,-r*sin],
                    [0,1,r*cos,sin,r*cos]])
        
        Sigma=np.diag([0,0,self.stita**2,self.sr**2,self.sfi**2])
        Sigma[0:2, 0:2] = cov_a         
        return np.matmul(np.matmul(J,Sigma),J.T)
    
    def get_prob_from_cov_n(self,cov):
        """
        Retrieve the probability of hyperrectangle multivariate gaussian distribution
        Calcula los autovalores de la matriz de covarianza, luego calcula la probabilidad en cada eje y luego multiplica. Leon Garcia pag 324/833
        (el libro dice que si son independiente la acumulada conjunta es el producto de las acumuladas)
        """
        w,v=eig(cov) # w is eigenvalues and v is eigenvectors
        cs2=self.cell_size/2
        limits_=np.array([0, cs2])
        p_=1
        #import pdb;pdb.set_trace()
        for i in range(len(w)):
            acumulada=norm_distribution.cdf(x=limits_,cov=w[i],mean=0)
            p_=p_*(acumulada[1]-acumulada[0])*2 # check this operation.
        return p_
        
    def create_uncertainty_fov(self):
        '''
        Create a boolean rings of FOV
        It doesn't need input parameters
        Retrieve a list called: fov_rings, each element is a ring of FOV
        '''
        # 
        (m,n)=self.exploration_FOV.shape
        # Crear la matriz de anillos
        x, y = np.meshgrid(np.arange(m), np.arange(n))
        #self.radious=np.arange(0,self.FOV,1)
        self.radious=np.linspace(0, self.FOV, int(self.FOV) + 1)
        self.fov_rings=[]
        s=self.cell_size
        for i,radio in enumerate(self.radious):           
            rma =self.radious[i+1]/s; rmi = self.radious[i]/s; 
            distancia_centro = np.sqrt((x - (m - 1) / 2) ** 2 + (y - (n - 1) / 2) ** 2)
            self.fov_rings.append(np.logical_and(distancia_centro >= rmi, distancia_centro <= rma).astype(int))
            if i==len(self.radious)-2:
                break
        
    def uncertainty_fov(self,cov_a):
        """
        Esta funcion crea el fov de incertidumbre a partir de la covarianza de pose y de la medicion
        $4
        """
        (m,n)=self.exploration_FOV.shape
        u_fov=np.full(shape=[m,n],fill_value=0.0)
        for i,radio in enumerate(self.radious): 
            cov_=self.Sigma_measure(cov_a,0,self.radious[i],0)
            p=self.get_prob_from_cov(cov_)
            u_fov=u_fov+p*self.fov_rings[i]
            if i==len(self.radious)-2:
                break
            
        return u_fov
                       
    def build_map_msg(self):
        """
        build map for send to server map
        """
        #mapp=self.landmarks_map
        #mapp=self.exploration_map
        # mapp_=copy(self.full_map) 
        mapp_=copy(self.exploration_map)        
        p_=np.flip(mapp_,axis=0)
        #import pdb; pdb.set_trace()

        m=map_msg()        
        #m.header.frame_id='/robot1_tf/odom_groundtruth'
        m.header.frame_id=self.center_frame
        #import pdb;pdb.set_trace()
        m.info.origin.position.x=self.map_size[0]*self.cell_size/2
        m.info.origin.position.y=-self.map_size[1]*self.cell_size/2
        m.info.origin.orientation.z=0.7071068
        m.info.origin.orientation.w=0.7071068
        m.info.resolution=self.cell_size
        m.info.width=self.map_size[0]
        m.info.height=self.map_size[1]
        #m.info.origin
        #import pdb;pdb.set_trace()

        #mi=np.min(mapp); mx=np.max(mapp-mi)
        #if mx<= 0:
        #    mx=1
        #aux=(100*np.reshape(1-(mapp-mi)/mx,(1,self.map_size[0]*self.map_size[1]))).astype(int)
        #aux=(100*np.reshape(mapp,(1,self.map_size[0]*self.map_size[1]))).astype(int)
        
        # map linealization
        #b=0.717;a=0.416
        #c_=np.log(-np.sqrt(p_) +1)
        #a_=a*self.cell_size**2/4;b_=b*self.cell_size/2
        #aux=np.reciprocal((-b_+np.sqrt(b_**2-4*a_*c_))/(2*a_))

        #aux=(self.cell_size/(3.464101615137754))*np.reciprocal(np.power(p_,0.5)) #3.464101615137754 =2*sqrt(3)
        unknown_idx=p_==self.beta
        aux=self.a*np.reciprocal(np.power(p_,0.5)) # sigma
        mi=np.min(aux); mx=np.max(aux-mi)
        #mx=np.pow(self.sigma_max2,0.5)
        if mx<= 0:
            mx=1
        #import pdb; pdb.set_trace()
        aux=(100*(aux-mi)/mx)
        aux[aux<1]=1 # el 1 es para que no quede saturado en 0
        aux[unknown_idx]=-1 # para identificar el  espacio desconocido
        aux=(np.reshape(aux,(1,self.map_size[0]*self.map_size[1]))).astype(int)
       
        # end map linealization
        
        aux=aux.tolist()[0]        
        m.data=aux
        self.m=m

    def build_map_msg2(self,map_,automatic_theshold=True):
        """ Generic implementation for build map msg

        input:
            .- map_ is the map to send
            .- automatic_theshold: when is true, the values are comprimed between 0 and 1 
        output:
            .- m is the map_msg
        """
        mapp=np.flip(map_,axis=0)
        #import pdb; pdb.set_trace()

        m=map_msg()        
        #m.header.frame_id='/robot1_tf/odom_groundtruth'
        m.header.frame_id=self.center_frame
        #import pdb;pdb.set_trace()
        m.info.origin.position.x=self.map_size[0]*self.cell_size/2
        m.info.origin.position.y=-self.map_size[1]*self.cell_size/2
        m.info.origin.orientation.z=0.7071068
        m.info.origin.orientation.w=0.7071068
        m.info.resolution=self.cell_size
        m.info.width=self.map_size[0]
        m.info.height=self.map_size[1]
        unknown_idx=mapp==0.5
        #m.info.origin
        #import pdb;pdb.set_trace()
        if automatic_theshold:            
            mi=np.min(mapp); mx=np.max(mapp-mi)
            if mx<= 0:
                mx=1
            aux=(100*np.reshape(1-(mapp-mi)/mx,(1,self.map_size[0]*self.map_size[1]))).astype(int)
        else:
            aux=100*mapp
            aux[unknown_idx]=-1
            aux=(np.reshape(aux,(1,self.map_size[0]*self.map_size[1]))).astype(int)
        #aux=(100*np.reshape(mapp,(1,self.map_size[0]*self.map_size[1]))).astype(int)
        aux=aux.tolist()[0]        
        m.data=aux
        return m
    
    def build_exploration_map(self):
        """ Get pose, FOV and uncertainty for build the map of exploration
        a partir del campo de vision, genera las actualizaciones al mapa de exploracion

        inputs:
            .- self.states: pose of robot
            .- self.FOV: field of view of robot
            .- self.P: covariance matrix of robot
        outputs:
            .- self.exploration_map: map of exploration

        TODO: Hay que comentar o separar en partes esta funcion, es muy compleja y larga.
        """
        # Calculo de inceridumbre del agente
        sxx=self.P[0][0]
        syy=self.P[1][1]
        sxy=self.P[0][1]
        A=np.array([[sxx, sxy],[sxy,syy]]) 
        uncertainty_fov=self.uncertainty_fov(A)
        p_a=self.get_prob_from_cov_n(A)# probabilidad del agente devido a su incertidumbre
        # Fin calculo de inceridumbre del agente

        self.current_uncertainty=p_a
        #exp=mask*np.max(big_mat)#+ ~mask*self.alpha
        #import pdb;pdb.set_trace()
        #self.RC.orientation=self.ori.orientation
        #self.RC.get_raycasting_mask3() # Va atrasado... no se porque
        mask=copy(self.mask)#self.RC.mask_raycasting
        exp=mask*uncertainty_fov+ ~mask*self.beta # donde no se ve pone beta, donde se ve pone la incertidumbre de medicion propagada

        (a,b)=self.exploration_FOV.shape    
        cz=self.cell_size
        px=int(self.states[0]/cz+self.map_size[0]/2)
        py=int(self.states[1]/cz+self.map_size[1]/2)    
        low_x=int(px-a/2);low_y=int(py-b/2)
        old_map=self.exploration_map[low_x:low_x+a,low_y:low_y+b]
        lo_lm=self.prob2logodds(exp)
        lo_old_map=self.prob2logodds(old_map)
        lo_updated=self.bayesian_update_exploration(lo_new=lo_lm,lo_old=lo_old_map)
           
        prob_updated=self.logodds2prob(lo_updated)    
        aux=copy(self.exploration_map_bool)
        aux[low_x:low_x+a,low_y:low_y+b]=mask
        self.exploration_map[aux]=prob_updated[mask]# El truco de usar mascaras es disminuir el costo computacional.
        #ValueError: NumPy boolean array indexing assignment cannot assign 6454 input values to the 6676 output values where the mask is true
        # Este error salta xq el tamaño de la mascara es menor que el tamaño de la matriz de exploracion o al reves. 
    
    def pub_map(self,map_,topic_):
        """ Generic algorithm for to send map to ROS

         Send a list of floats with a Float32MultiArray message type 
         Input:
            .- map_: map to send, np.array
            .- topic_: topic publisher
        """
        s=map_.shape
        a=MultiArrayDimension()
        a.label="height"
        a.size=s[0]
        a.stride=s[0]*s[1]
        b=MultiArrayDimension()
        b.label="width"
        b.size=s[1]
        b.stride=s[1]
        data=states_()
        data.layout.dim=[a,b]
        # TODO: esta linea deberia ser muy costosa, ya que se hace 600 veces o mas
        big_list=[]
        for i in map_.astype(np.float32).tolist():
            big_list.extend(i)
        
        data.data=big_list
        #import pdb; pdb.set_trace()
        topic_.publish(data)
        
    def get_raycasting_from_obstacle_map(self):
        """ Get raycasting from obstacle map
        
        If there is not a laser data, the raycasting is calculated from the obstacle map
        
        Input:
            .- self.obstacle_map: np.array with the obstacle map
        Output:
            .- self.RC.mask_raycasting: bool np.array with the raycasting
        #TODO: very slow, improve the algorithm.  it is the most expensive part of the code
        """
        
        
        cz=self.cell_size;#mask=self.exploration_map_bool
        px=int(self.states[0]/cz+self.map_size[0]/2)
        py=int(self.states[1]/cz+self.map_size[1]/2)
        (a,b)=self.exploration_FOV.shape        
        low_x=int(px-a/2);low_y=int(py-b/2)       
        mask=(self.obstacle_map[low_x:low_x+a,low_y:low_y+b]>0.6)# a square of FOV size with obstacles
        self.RC.get_ray_casting_mask2(mask) # ray casting nuevo
        #mask=(mask * self.exploration_FOV)        
        #self.RC.get_laser_data_for_obstacle_map(mask) # ray casting viejo
        #self.RC.orientation=0.0
        #self.RC.get_raycasting_mask()
        #self.RC.scan_offline()               
            
    def run(self):
        """this code run a principal loop but without ROS
        The data passed is trough set variables in the object

        Only do one loop.
        
        Data required is the same for get_states function:
        .- get_states(self,data=states_()):
        .- data in self.full_data=states_() is a list of floats with the following order:
            .- [states,1^st row of covariance matrix,2^nd row of covariance matrix,..., n^th row of covariance matrix]
        
            Principalmente se usa para hacer las alucinaciones.
        """
        self.get_states()
        #self.process_landmark() # parece que esto no hace falta
        self.get_raycasting_from_obstacle_map() #
        self.build_exploration_map()
        #self.map_fusion() # copiar exploration map a full map
                
    def _run_debug(self):
        rospy.init_node('map_manager',anonymous=True)
        rospy.loginfo('Map manager initialized')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        self.time=1.0
        self.rate = rospy.Rate(1/self.time) #hz
        while not rospy.is_shutdown(): 
            if self.new_data_:
                if  self.new_laser_data:
                
                    #self.get_uncertainty_pose()
                    #t=[]
                    #t.append(time.time())
                    #self.process_landmark() # is the most spending time routine # parece que esto no hace falta
                    #t.append(time.time())
                    self.build_exploration_map() 
                    #t.append(time.time())
                    # self.map_fusion() # copiar exploration map a full map
                    #t.append(time.time())
                    #self.new_data_=True
                    ##########################################
                    # Publicar divergencia
                    msg=divergence_msg()
                    msg.data=self.get_divergence()                
                    self.publish_divergence.publish(msg)
                    ##########################################
                    #for i in range(len(t)-1):
                    #    print('costo computacional paso: ',i,' tiempo ',t[i+1]-t[i])
                    self.pub_map(self.exploration_map,self.exploration_map_publisher)
                    # self.pub_map(self.landmarks_map,self.landmarks_map_publisher)
                    self.pub_map(self.obstacle_map,self.obstacle_map_topic_matrix)
                    #self.pub_expl_map()
                    self.new_data_=False
                    #self.new_laser_data=False

            #import pdb;pdb.set_trace()
            ##########################################
            # q:¿QUe diferencia hay entre estos mensajes y los que estan dentro del if?
            # a: me parece que los pub de arriba mandan una matriz y los pub de abajo mandan el mapa para mostrarlo en rviz 
            self.build_map_msg()
            self.map_topic.publish(self.m)  
            # # obstacle map
            self.obstacle_map_topic.publish(self.build_map_msg2(self.obstacle_map,False))  
            self.rate.sleep()                      
        
if __name__=='__main__':
    time.sleep(7)
    n=len(sys.argv)
    sb=rospy.get_param('sigma_max')
    fov=rospy.get_param('fov')
    cell_size=rospy.get_param('cell_size')
    bbox=rospy.get_param('bbox')
    map_size_n=rospy.get_param('map_size')
    try:
        if n>2:
            mapeo_=MapServer(states_cova_topic=sys.argv[1],
                             center_frame=sys.argv[2],topic_map=sys.argv[3],
                             exp_map_topic=sys.argv[4],debug=True,
                             sb=sb,fov=fov,cell_size=cell_size,bbox=bbox,map_size_m=map_size_n)
            rospy.set_param('/beta',float(mapeo_.beta))
            mapeo_._run_debug()# run with ROS and begin a node of ROS
        else:
            mapeo_=MapServer(sb=sb,fov=fov,cell_size=cell_size)
            rospy.set_param('/beta',float(mapeo_.beta))
            mapeo_.run()
        
    
    except rospy.ROSInterruptException:
        pass
    

