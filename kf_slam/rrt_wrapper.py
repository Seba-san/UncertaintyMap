#!/usr/bin/env python3
"""
mediante un topic de ROS conecta el rrt star con el mapa de ocupacion. Devuelve el camino optimo para cada objetivo planteado.
Mantiene el arbol generado,si no se quiere mantener hay que poner la bandera rst_map=True.

"""
import rospy
import numpy as np
from GET_data import GET_data
from nav_msgs.msg  import Path
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from scipy.ndimage import distance_transform_edt # implementa la SDF sobre un mapa de ocupacion.
from rrt_planning import RRTStar
from raycasting import RayCasting

import matplotlib.pyplot as plt

class Landmarks:
    def __init__(self):
        self.P=[]
        self.landmarks_poses=[]

class RRT_star_ROS:
    def __init__(self,minimun_distance=1.0,show_animation=False,max_iter=100):
        """_summary_

        Args:
            minimun_distance (float, optional): distancia minima a los obstaculos. Defaults to 1.0.
            show_animation (bool, optional): muestra el path final en una imagen. Defaults to False.
            max_iter (int, optional): cantidad de puntos generados. No necesariamente son nodos. Defaults to 100.
        """
        self.compute=False # Flag for compute the path
        self.valid_path=False # Flag to send a valid path
        self.cell_size=0.1 # cell size in meters
        self.minimun_distance=minimun_distance     # in meters     
        self.step_size=1.0# in meters (antes en 1.0) $1
        self.max_iter=max_iter
        self.nearest_distance=5.0 # in meters distancia donde hace la busqueda del nodo mas cercano

        self.run_hot_flag=False # It doesn't recompute all tree again.
        self.robot_pose=PoseStamped().pose.position
        self.goal_pose=PoseStamped().pose.position

        self.show_animation = show_animation
        self.path=None
        self.landmarks=Landmarks()
        self.Q=0.01 # es la incertidumbre de la odometria para un paso de 0.3m. Ver como sacarlo de rosparams... TODO $1 !!!
        self.rc=RayCasting()

    def get_obstacle_map(self):
        gt_data=GET_data()
        while not gt_data.obstacle_map.flag_ or not gt_data.got_states :
            pass # wait to get the obstacle map
        
        # self.obstacle_map=np.flip(gt_data.obstacle_map.map_,1)
        self.obstacle_map=gt_data.obstacle_map.map_.T
        self.landmarks.landmarks_poses=gt_data.states[2:]
        self.landmarks.P=gt_data.P
        del gt_data
        self.process_landmark()

    def process_landmark(self):
        """ Arma una lista self.landmarks_list=[l1, l2, ...] donde l1=[x,y,sigma_promedio] con las posiciones de los landmarks y la matriz de covarianza.
        """
        
        #import pdb;pdb.set_trace()
        landmarks=[]
        K=1/(self.cell_size)
        sm=self.obstacle_map.shape[0]/2
        N=int(len(self.landmarks.P)/2)
        for i in range(1,N):
            D=np.array([[self.landmarks.P[2*i][2*i],self.landmarks.P[2*i][2*i+1]],[self.landmarks.P[2*i+1][2*i],self.landmarks.P[2*i+1][2*i+1]]])
            sigma=np.power(np.linalg.det(D),0.25)
            idx=2*i-2
            l_=self.landmarks.landmarks_poses[idx:idx+2]
            ll_ = [elemento * K + sm for elemento in l_]# lo pasa a celdas
            ll_.extend([sigma*K/self.Q]) # hay que transformar esto a "distancia"
            landmarks.append(ll_)
        
        self.landmarks_list=landmarks
        
        
    def process_obstacle_map(self,min_dist=None):
        """Devuelve el espacio transitable.
        Este espacio es a min_dist de los obstaculos y a unknown_distance de los espacios desconocidos hacia dentro.
        """
        unknown_distance=1.0 # in meters
        if min_dist is None:
            min_dist=self.minimun_distance
        sdf_map = distance_transform_edt(~(self.obstacle_map>0.5)) # 0.6 es el umbral de ocupacion
        unknown_space = distance_transform_edt(~(self.obstacle_map<0.5)) # 0.6 es el umbral de ocupacion                
        return (sdf_map>min_dist/self.cell_size) & (unknown_space<(unknown_distance/self.cell_size)) # son TRUE solo el espacio libre y lejos de obstaculos
    
    def run_first_time(self):        
        #sdf_map = distance_transform_edt(~(self.obstacle_map>0.6)) # 0.6 es el umbral de ocupacion
        #import pdb; pdb.set_trace()
        #transit_map=(sdf_map>self.minimun_distance/self.cell_size) & (self.obstacle_map<0.2) # son TRUE solo el espacio libre y lejos de obstaculos

        LOS_map=self.rc.build_LOS_map(self.obstacle_map>0.4,self.landmarks_list)   
        if self.show_animation:
            plt.figure(1) 
            plt.imshow(LOS_map)
            plt.figure(2)
            plt.imshow(self.obstacle_map>0.4)
            plt.show()  

        #import pdb; pdb.set_trace()         

        transit_map=self.process_obstacle_map()
        self.shape_=[transit_map.shape[0]/2,transit_map.shape[1]/2]
        #import pdb; pdb.set_trace()
        
        sigma_robot=self.robot_pose.z/(self.cell_size*self.Q) 
        r_pose=(self.robot_pose.x/self.cell_size+self.shape_[0],self.robot_pose.y/self.cell_size+self.shape_[1]) # robot pose en celdas
        g_pose=(self.goal_pose.x/self.cell_size+self.shape_[0],self.goal_pose.y/self.cell_size+self.shape_[1]) # goal pose en celdas
        #rrt_module=RRTStar(start=r_pose,goal=g_pose,obstacle_map=transit_map,step_size=1.0/self.cell_size,max_iter=1000,nearest=10/self.cell_size)
        self.rrt_module=RRTStar(start=r_pose,goal=g_pose,obstacle_map=transit_map,step_size=self.step_size/self.cell_size,
                                max_iter=self.max_iter,nearest=self.nearest_distance/self.cell_size,animation=self.show_animation,
                                landmarks=self.landmarks_list,temperature=sigma_robot,LOS_map=LOS_map)
        #self.rrt_module.deep_debug=True # es para que muestre el arbol completo
        path_=self.rrt_module.plan()
        self.path_manager(path_)

    def run_hot(self):
        """
        Run RRT star algorithm with the same tree.
        """
        self.run_hot_flag=False
        g_pose=(self.goal_pose.x/self.cell_size+self.shape_[0],self.goal_pose.y/self.cell_size+self.shape_[1]) # goal pose en celdas
        state=self.rrt_module.is_connected(g_pose)
        if state is None:
            # hacer correr el rrt star con el mismo arbol
            #print('El punto objetivo no esta conectado al arbol')
            #plt.show()
            self.valid_path=False
            self.compute=False 
            #print('Se computa el arbol nuevamente sobre el anterior')
            #path_=self.rrt_module.plan()
            #self.path_manager(path_)
        else:
            # devuelve el path
            #print('Camino encontrado sin computar nuevamente')
            self.path_manager(state)

    def path_manager(self,path_):
        """
        Maneja el path obtenido, si es valido lo publica en un topic.
        """
        #if path_:
        #    print("Camino encontrado:")
        #    print(path_)
        #    print("Costo")
        #    print(self.rrt_module.nodes[-1].cost)
        #else:
        #    print("No se pudo encontrar un camino.")

        if path_:
            path = np.array(path_)
            if self.show_animation:
                print(path)
                plt.plot(path[:,0], path[:,1], 'r--')
                plt.xlabel("X")
                plt.ylabel("Y")
                plt.show()

            self.path=Path()
            #self.path.header
            for l in path_:
                p=PoseStamped()
                p.pose.position.x=(l[0]-self.shape_[0])*self.cell_size
                p.pose.position.y=(l[1]-self.shape_[1])*self.cell_size
                self.path.poses.append(p)
                        
            self.valid_path=True 
            self.compute=False
        else:
            if self.show_animation:
                plt.show()
            self.valid_path=False
            self.compute=False   
            self.path=None  



    def callback_request(self,msg=PoseArray()):
        """recibe el origen y el objetivo. Devuelve el path con RRT star de forma asincronica en otro topic."""
        if self.compute:
            rospy.loginfo('computing path...')
            return
        
        if (msg.poses[0].position.x==self.robot_pose.x and msg.poses[0].position.y==self.robot_pose.y):
            self.run_hot_flag=True
        else:
            self.robot_pose=msg.poses[0].position
            # hay que dar vuelta el robot pose
            #x=self.robot_pose.x; y=self.robot_pose.y
            #self.robot_pose.x=y; self.robot_pose.y=x

        self.goal_pose=msg.poses[1].position
        # hay que dar vuelta el goal pose
        #x=self.goal_pose.x; y=self.goal_pose.y
        #self.goal_pose.x=y; self.goal_pose.y=x
        self.compute=True               
        
if __name__=='__main__':
    try:
        minimun_distance=rospy.get_param('min_obstacle_distance_sdf')
        max_iter_=rospy.get_param('max_iter_rrt')
    except:
        minimun_distance=0.4
        max_iter_=1000
    
    
        
    rrt_star=RRT_star_ROS(minimun_distance=minimun_distance,show_animation=True,max_iter=max_iter_)# antes max_iter=1000 $1
    
    rospy.init_node('rrt_star_ros_node', anonymous=True)    
    request_=rospy.Subscriber('/planning_route_rrt_star',PoseArray,rrt_star.callback_request,queue_size=1)
    publisher_=rospy.Publisher('/path_rrt_star',Path,queue_size=1)
    rrt_star.cell_size= rospy.get_param('/cell_size')    
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        if rrt_star.compute:  
            #import pdb; pdb.set_trace()
            if not rrt_star.run_hot_flag: 
                rrt_star.get_obstacle_map()                                                 
                rrt_star.run_first_time()                
            else:
                rrt_star.run_hot()
            
        if rrt_star.valid_path:
            rrt_star.valid_path=False            
            publisher_.publish(rrt_star.path)
            rospy.loginfo('publishing path')
        
        rate.sleep()
    
    if rospy.is_shutdown():
        rospy.loginfo('shutting down rrt star node')
    