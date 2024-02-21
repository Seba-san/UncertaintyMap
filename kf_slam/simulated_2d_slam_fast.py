"""
La idea de esta funcion es implementar una prediccion lo mas rapido posible, sin mucho costo computacional
Inputs:
    - M: Mapa
    - P: Matriz de covarianza
    - X: Estados del sistema
    - FOV: Campo de vision
    - parameters: [cellsize,center,Q0,beta,sigma_mx]
output:
    - divergencia

warnings:
    - No es capaz de realizar cierres de lazos. No actualiza las estimaciones de los landmarks debido a un cierre de lazo.
"""
import numpy as np
from raycasting import RayCasting
class FastEstimation:
    def __init__(self, M, P, X, FOV,parameters,obstacle_map):
        
        self.P = P
        self.X = X
        self.FOV = FOV #meters
        self.parameters = parameters # [cellsize,center,Q0,beta,sigma_mx]
        self.make_mask_FOV()
        self.k=0.0
        l=0
        self.V0=np.sqrt(self.P[2*l,2*l])
        self.M = self.sigma_map_from_p_map(M)
        self.E_map=M
        self.RC=RayCasting()
        self.RC.FOV=FOV
        self.RC.make_mask_FOV()
        self.obstacle_map=obstacle_map
        self.a=parameters[4] # constante para obtener sigma map
        self.GAIN=1.5
    
    def Gl(self):
        # Valor dado por la existancia de un landmark
        
        L=self.X.size
        r=self.X[0][0:2]
        Gl_=np.zeros(int((L-2)/2))
        #import pdb; pdb.set_trace()
        for l in range(int((L-2)/2)): # se elimina la pose del robot inicial   
            l=l+1         
            land = self.X[0][2*l:2*l+2]
            if np.linalg.norm(land-r)<self.FOV:
                Gl_[l-1]=np.sqrt(self.P[2*l,2*l])*self.GAIN# Splo se toma una componente $1 OJO ACA!
            else:
                Gl_[l-1]=1000  

        result=np.min(Gl_)

        if result<1000:
            self.V0=result
            self.k=0.0
        return result

    def Ge(self):
        # Valor dado por el modelo de incertidumbre del robot
        return self.V0+self.parameters[2]*(self.k)

    def get_M_update(self,X):
        """ Update map with new slam state"""
        self.X=X # slam state
        s_l=self.Gl()
        s_r=self.Ge()
        d=min(s_l,s_r)
       
        # get all data map in FOV
        r=self.X[0][0:2]
        cz=self.parameters[0] # cellsize
        center=self.parameters[1] # center
        cr=(r/cz+center).astype(int)
        fov_c=int(self.FOV/cz)
        M_view=self.M[cr[0]-fov_c:cr[0]+fov_c,cr[1]-fov_c:cr[1]+fov_c]
        # --------------  update map
        ## raycasting
        mask=(self.obstacle_map[cr[0]-fov_c:cr[0]+fov_c,cr[1]-fov_c:cr[1]+fov_c]>0.6)*self.exploration_FOV
        self.RC.get_ray_casting_mask2(mask)
        #data_m=M_view[self.exploration_FOV]
        data_m=M_view[self.RC.mask_raycasting]
        data_updated=np.minimum(data_m,d)
        M_view[self.RC.mask_raycasting]=data_updated
        self.M[cr[0]-fov_c:cr[0]+fov_c,cr[1]-fov_c:cr[1]+fov_c]=M_view
        self.k=self.k+1.0
        # Sigma del robot
        #self.M[0,0]=s_r
        self.M_p=self.p_map_from_sigma_map(self.M)
        #import pdb; pdb.set_trace()
        return self.get_divergence(self.M_p) 
    
    def get_divergence(self,M):
        return self.get_divergence_e_map(M)

    def get_divergence_e_map(self,E_map):
        E=E_map
        N=2;beta=self.parameters[3]
    
        Dkl=(self.parameters[0]**2)*(np.log(E / beta) - N / 2 + (N / 2) * (beta / E) ** (2 / N))
        idx=E<beta
        Dkl[idx]=-Dkl[idx]
        Dkl_tot=np.sum(Dkl)
        return Dkl_tot       
    
    def get_divergence_(self,M):
           """
           get divergence from initial map and current map
           """
           # sum p log p/self.alpha
           alpha=self.parameters[3]
           a=np.multiply(M,1/alpha)        
           b=np.multiply(M,np.log(a))
           #import pdb; pdb.set_trace()
           #print(np.sum(b[b<0])," only negative values of divergence")
           #print(np.sum(b),"divergence")
           divergence=np.sum(b)
           return divergence

    def make_mask_FOV(self):
        """ make a mask of field of view to generate map of exploration
        Only make once
        """
        r=self.FOV
        cz=self.parameters[0] # cellsize
        a=int(2*r/cz); b=a
        self.exploration_FOV=np.full(shape=[a,b],fill_value=False)
       
        for i in range(a):
            for k in range(b):
                x=float(i)*cz
                y=float(k)*cz
                if (x-r)**2+(y-r)**2<r**2:
                    self.exploration_FOV[i,k]=True
    
    def p_map_from_sigma_map_old(self,M):
        b=-0.717;a=-0.416; # gaussian cdf aproximation
        cz=self.parameters[0] # cellsize
        x=cz/(2*M)
        beta_=(1-np.exp(b*x+a*x**2))**2
        return beta_
    
    def p_map_from_sigma_map(self,M):
        E=np.power(self.a/M,2)
        return E
        b=-0.717;a=-0.416; # gaussian cdf aproximation
        cz=self.parameters[0] # cellsize
        x=cz/(2*M)
        beta_=(1-np.exp(b*x+a*x**2))**2
        return beta_

    def sigma_map_from_p_map_old(self,M):
        """ Convert a map of probability to a map of sigma
        """
        # map linealization        
        b=0.717;a=0.416
        p_=M
        c_=np.log(-np.sqrt(p_) +1)
        cz=self.parameters[0] # cellsize
        mz=np.size(M)#map size
        a_=a*cz**2/4;b_=b*cz/2
        M_inv=np.reciprocal((-b_+np.sqrt(b_**2-4*a_*c_))/(2*a_))
        #M_inv=(100*np.reshape(aux,(1,mz[0]*mz[1]))).astype(int)
        # end map linealization
        return M_inv
    
    def sigma_map_from_p_map(self,M):
        """ Convert a map of probability to a map of sigma
        """
        sigma=self.a/np.power(M,0.5)
        return sigma
     
        # map linealization        
        b=0.717;a=0.416
        p_=M
        c_=np.log(-np.sqrt(p_) +1)
        cz=self.parameters[0] # cellsize
        mz=np.size(M)#map size
        a_=a*cz**2/4;b_=b*cz/2
        M_inv=np.reciprocal((-b_+np.sqrt(b_**2-4*a_*c_))/(2*a_))
        #M_inv=(100*np.reshape(aux,(1,mz[0]*mz[1]))).astype(int)
        # end map linealization
        return M_inv

    def get_M(self):
        return self.p_map_from_sigma_map(self.M)
    
    
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
        mask=(self.obstacle_map[low_x:low_x+a,low_y:low_y+b]<0.6)# a square of FOV size with obstacles
        mask=(mask * self.exploration_FOV) 
        self.RC.get_laser_data_for_obstacle_map(mask)
        self.RC.orientation=0.0
        self.RC.scan_offline()   

        
        