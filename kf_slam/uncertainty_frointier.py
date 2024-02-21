# from map_server import MapServer
from copy import deepcopy as copy
import numpy as np
# from simulated_2d_slam2 import GET_data
import matplotlib.pyplot as plt

import rospy
import time
from sklearn.cluster import KMeans


class UFrontier:
    """
    Esta clase implementa el algoritmo de búsqueda de frontera de incertidumbre
    """
    def __init__(self,fov=5.0,cs=0.1,beta=1.0,sig_mx=1.0,uf_treshold=0.2,show_animation=False):
        self.FOV=fov # field of view
        self.cell_size=cs # cell size
        self.beta=beta
        self.umbral_ddivergencia=uf_treshold
        self.sig_mx=sig_mx
        self.a=np.power(self.beta,0.5)*self.sig_mx # line 64 map_server
        self.show_animation=show_animation
        # self.show_animation=True # $1 ojo aca
        self.make_mask_FOV()              
    
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
    
    def set_maps(self,u_map=np.array([]),obstacle_map=np.array([])):
        """
        asocia los mapas al objeto
        """        
        #self.fullmap=copy(M)
        self.u_map=copy(u_map)
        self.obstacle_map=copy(obstacle_map)
        self.sigma_map=self.a*np.reciprocal(np.power(self.u_map,0.5))
        #self.get_sigma_map()
    
    def find_frontiers(self,pose=np.array([])):
        """
        a partir de la pose del agente, devuelve la frontera de incertidumbre más cercana la distancia y las filas y columnas
        donde se encuentra el punto de la frontera.
        pose: [x,y] es la posicion del vehiculo en metros
        devuelve: 
        p_objetive: [f,c] es la posicion de las coordenadas del mapa
        distance: distancia en metros
        """
        n,m=self.sigma_map.shape
        cx=n/2 # center of map
        self.sigma_map[self.sigma_map>self.sig_mx]=self.sig_mx # Para evitar usar umbrales con sigmas mayores a sig_mx
        pose=np.array([[pose[0]/self.cell_size+cx,pose[1]/self.cell_size+cx]]) # pose in cells
        d1=np.diff(self.sigma_map,axis=0);d1=d1[:,:n-1]
        d2=np.diff(self.sigma_map,axis=1);d2=d2[:n-1,:]
     
        d=np.power(np.power(d1,2)+np.power(d2,2),0.5)
        
        #fig, ax = plt.subplots()
        #cax1 = ax.imshow(d)  # El valor alpha controla la transparencia
        #fig2, ax2 = plt.subplots()
        #cax2=ax2.imshow(self.obstacle_map)
        #plt.show()
        if self.show_animation:
            plt.figure(1)
            plt.imshow(d)
            plt.title('gradientes de incertidumbre')
            #plt.show()
            plt.figure(2)
            plt.imshow(self.obstacle_map>0.6)
            plt.title('obstaculos')
            plt.show()
            


        idx=self.obstacle_map[:n-1,:n-1]>0.6 # umbral arbitrario de obstaculos
        d[idx]=0

       

        idx=d>self.umbral_ddivergencia # umbral arbitrario de salto de incertidumbre
        d[idx]=1
        idx=d<self.umbral_ddivergencia # umbral arbitrario de salto de incertidumbre
        d[idx]=0
        
        #fig3, ax3 = plt.subplots()
        #cax3 = ax3.imshow(d)  # El valor alpha controla la transparencia
        if self.show_animation:
            plt.imshow(d)
            plt.title('fronteras de incertidumbre')
            plt.show()
        #plt.show()
        #import pdb; pdb.set_trace()
        
        f,c=np.where(d==1); loc=np.array([f,c])

        if loc.shape[1]==0:
            print('no frontiers found')
            return np.array([]),np.array([]),False
        
        n_clusters=1
        ms=self.obstacle_map.shape[0] # map shape
        while True:
            kmeans = KMeans(n_clusters=n_clusters,n_init='auto')
            kmeans.fit(loc.T)
            assign = kmeans.labels_
            flag=0
            for i in range(n_clusters):                
                desvio_total=np.sqrt(np.var(loc[0,assign==i])+np.var(loc[1,assign==i]))
                if desvio_total>self.FOV/(2*self.cell_size): # Tiene que tener menos varianza que la mitad del fov
                    n_clusters+=1
                    flag=1
                    break
                
                
                objetives=np.round(kmeans.cluster_centers_).astype(int)    
                # 
                # if self.show_animation:
                #    plt.imshow(self.obstacle_map>0.6)
                #    for i in range(n_clusters):
                    #  plt.plot(objetives[i,1],objetives[i,0],'x')
                    #  plt.plot(objetives[i,1]+1,objetives[i,0]+1,'rx')
                    #  plt.plot(objetives[i,1]-1,objetives[i,0]-1,'bx')
                #    plt.title('objetivos de exploracion')
                #    plt.show()
                        #    
                # import pdb; pdb.set_trace()
                if (np.any(self.obstacle_map[objetives[:,0]+1,objetives[:,1]+1]>0.6) or
                     np.any(self.obstacle_map[objetives[:,0]-1,objetives[:,1]-1]>0.6)): # si algun objetivo cae dentro de un obstaculo, se vuelve a calcular

                    #self.show_animation=True
                    print('Frontiers over obstacles')
                    #import pdb;pdb.set_trace()
                    idx=np.where(self.obstacle_map[objetives[:,0],objetives[:,1]]>0.6)
                    kmeans.cluster_centers_=np.delete(kmeans.cluster_centers_,idx,axis=0)
                    if kmeans.cluster_centers_.shape[0]==0:
                        n_clusters+=1
                        flag=1
                        break                    
                
            if flag==0:
                break
        
        centers=(kmeans.cluster_centers_-cx)*self.cell_size
        if self.show_animation:
            plt.imshow(self.obstacle_map>0.6)
            for i in range(n_clusters):
                plt.plot(objetives[i,1],objetives[i,0],'x')
            
            plt.title('objetivos de exploracion puntuales')
            #import pdb; pdb.set_trace()
            plt.show()
            
        
        if self.show_animation:
            #plt.imshow(d)
            plt.imshow(self.obstacle_map>0.6)
            for i in range(n_clusters):
                plt.plot(loc[1,assign==i],loc[0,assign==i],'x')
            plt.title('objetivos de exploracion completo')
            plt.show()
        #self.show_points(loc[:,assign==0])
        #
        #kmeans.cluster_centers_
        
       
        # puede que un centro te quede en una zona no alcanzable, hay que hacer ese test o garantizar que eso no pase.

        
        #dist=np.power(np.power((loc-pose.T),2).sum(axis=0),0.5)
        #mask = dist >= self.FOV*2/self.cell_size # se eliminan los puntos que estan muy cerca, ver como levantar parametros $1, depende de simple o double
        #mask = dist >= 0 # se eliminan los puntos que estan muy cerca, ver como levantar parametros $1, depende de simple o double
        #dist=dist[mask] 
        
        if centers.shape[0]==0:
            print('no frontiers found')
            return np.array([]),np.array([]),False
        else:
            print('Amount of frontiers found: ',centers.shape[0])
            return centers,np.array([]),True
            
        #f=f[mask];c=c[mask]
        idx=np.argmin(dist) # se toma el punto más cercano. TODO: aca hay que hacer kmeans...
        p_objetive=np.array([f[idx],c[idx]])
        distance=dist[idx]*self.cell_size
        #import pdb; pdb.set_trace()
        if self.show_animation:
            self.show(d,p_objetive,pose[0])
        #self.show(d,p_objetive,pose[0])
        p_objetive_meters=(p_objetive-cx)*self.cell_size
        return p_objetive_meters,distance,True
    
    def show(self,d,p,pose):
        """map, objetive point, pose robot """
        #import pdb; pdb.set_trace()
        plt.imshow(d, cmap='jet', interpolation='nearest') 
        plt.plot(p[1],p[0],'rx')
        plt.plot(pose[1],pose[0],'go')
        # plt.pause(0.1)
        plt.show()

    def show_points(self,loc):
        plt.plot(loc[0,:],loc[1,:],'rx')
        plt.show()
    
    def get_divergence(self,M):
        """
        get divergence from initial map and current map
        """
        # sum p log p/self.alpha
        a=np.multiply(M,1/(2*self.beta-self.beta**2))        
        b=np.multiply(M,np.log(a))
        #import pdb; pdb.set_trace()
        #print(np.sum(b[b<0])," only negative values of divergence")
        #print(np.sum(b),"divergence")
        divergence=np.sum(b)
        return divergence
    
    def get_aprior_divergence(self,p_objetive=np.array([])):
        """
        A partir de la posicion de la frontera de incertidumbre, se calcula la divergencia que tendria si el robot fuera teletransportado
        a esa posicion. Se hacen aproximaciones para calcular el FOV y el nuevo mapa.
        """
        phantom_map=copy(self.fullmap)
        l=int(np.ceil(self.FOV/self.cell_size))
        M_masked= phantom_map[p_objetive[0]-l:p_objetive[0]+l,p_objetive[1]-l:p_objetive[1]+l]
        mx=np.max(phantom_map[p_objetive[0]-1:p_objetive[0]+2,p_objetive[1]-1:p_objetive[1]+2])
        # TODO: la siguiente linea hay que actualizarla con bayesian update
        phantom_map[p_objetive[0]-l:p_objetive[0]+l,p_objetive[1]-l:p_objetive[1]+l]=np.maximum(self.exploration_FOV*mx+~self.exploration_FOV*M_masked,M_masked)
        #self.plot(phantom_map)
        D=self.get_divergence(phantom_map)-self.get_divergence(self.fullmap)
        return D
    
    def get_sigma_map_(self):
        s=self.cell_size
        b=0.717;a=0.416
        c_=np.log(-np.power(self.fullmap,0.5) +1)
        a_=a*s**2/4;b_=b*s/2
        self.sigma_map=np.power((-b_+np.power(b_**2-4*a_*c_,0.5))/(2*a_),-1)
        
    def get_sigma_map(self):
        self.sigma_map=self.a*np.reciprocal(np.power(self.u_map,0.5))
        
        
    def run(self):
        p_objetive,distance=self.find_frontiers(pose=np.array([0,0])) 
        D=self.get_aprior_divergence(p_objetive)
        
    def plot(self,var):
        plt.imshow(var, cmap='jet', interpolation='nearest') 
        plt.show()
                                
if __name__=='__main__':
    uf=UFrontier(beta=0.00159022)
    gd=GET_data()
    admin=GET_data()
    while not (admin.got_states and admin.exp_map.flag_ and admin.land_map.flag_ and admin.obstacle_map.flag_): # wait for a states data
        print('waiting for data')
        print('states: ',admin.got_states, 'exp_map: ',admin.exp_map.flag_,'land_map: ',
              admin.land_map.flag_,'obstacle_map: ',admin.obstacle_map.flag_)            
        time.sleep(1)
    
    rospy.loginfo('data captured')
    
    states,P,exp_map,land_map,obstacle_map=admin.get_data() 
    M=exp_map+land_map-np.multiply(exp_map,land_map)
    uf.set_maps(M=M,obstacle_map=obstacle_map)
    p_objetive,distance,flag=uf.find_frontiers(pose=states[:2]) 
    if flag:
        D=uf.get_aprior_divergence(p_objetive)
    
    
    
        
    


        
    
        
        
        
        
       
        
        
        
          