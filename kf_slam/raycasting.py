#!/usr/bin/env python3
"""
This scripts is used to create a raycasting algorithm from LaserScan data
TODO: many methods are very slow, it is necessary to optimize them
"""
from sensor_msgs.msg import LaserScan as lasermsg
import numpy as np
from copy import deepcopy as copy
import rospy
from scipy.ndimage import binary_dilation
from scipy.ndimage import distance_transform_edt # implementa la SDF sobre un mapa de ocupacion.

import matplotlib.pyplot as plt
from scipy.signal import convolve2d
from scipy.ndimage import convolve

class RayCasting:
    def __init__(self,make_callback=False):
        self.debug=False # if True, publish a fake laser scan obtained from obstacle map
        self.data=lasermsg()
        self.orientation=0.0
        self.angles=np.linspace(-np.pi,np.pi,int(1+np.pi*2/0.008738775737583637))
        self.ang_step=0.008738775737583637 # radians
        self.FOV=5.0
        self.cell_size=0.1
        self.make_mask_FOV()
        self.mask_raycasting=copy(self.exploration_FOV)
        #rospy.init_node('raycasting',anonymous=True)
        if make_callback:
            topic_laser='/robot1/laser/scan_lidar_horizontal'
            self.laser_subs=rospy.Subscriber(topic_laser,lasermsg,self.angular_scan,queue_size=1)
        if self.debug:
            self.laser_pub=rospy.Publisher('/fake_laser',lasermsg,queue_size=1)
            
        #rospy.spin()
        # for test only $1
        #self.laser_pub=rospy.Publisher('/fake_laser',lasermsg,queue_size=1)
        #self.make_index_rad()
        
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
                if (x-r)**2+(y-r)**2<(r-self.cell_size)**2:
                    self.exploration_FOV[i,k]=True

    def make_index_rad(self):
        # Esta funcion esta en desuso. La idea es que genere los indices que relacionan el FOV con los angulos.
        #points=np.argwhere(self.exploration_FOV) 
        sh=self.exploration_FOV.shape

        self.angles_idx=(-1)*np.ones(shape=sh)
        for x in range(sh[0]):
            for y in range(sh[1]):
                if self.exploration_FOV[x,y]:  
                    x_=float(x)*self.cell_size-self.FOV
                    y_=float(y)*self.cell_size-self.FOV
                    angle=np.arctan2(y_,x_)
                    idx=np.argmin(abs(self.angles-angle))
                    self.angles_idx[x,y]=idx

        #import pdb;pdb.set_trace()
        return
        
    def angular_scan(self,data=lasermsg(),orientation=0.0):
        self.data=data
        self.orientation=orientation
        self.scan_offline()        
       
    

    def scan_offline(self):
        # Momentaneamente se usa esta intermediacion para evitar problemas de compatibilidad
        self.get_raycasting_mask()

    def get_raycasting_mask(self):
        """ Generate a mask of raycasting with True values in seen area
        
        Search the angles of laser scan that has a value greater than zero
        and then update the values behind the obstacle with False value.
        
        se hace en 2 pasos: 
         .- primero ubica los obstaculos en la matriz de FOV
         .- para evitar tener "filtraciones" se impleemnta una interpolacion. Esta interpolacion 
         evita que los rayos del lidar pasen entre dos obstaculos consecutivos pero diagonales. Para
         hacer esto usa la convolucion y un arreglo para detectar la situacion
         .- luego usa el metodo get_ray_casting_mask2 para generar el mapa de raycasting
        
        input:
            .- data: LaserScan message
            .- orientation: robot orientation
        output:
            .- mask_raycasting: mask of raycasting       
            
        TODO: revisar su funcionamiento
        """
        data=self.data
        orientation=self.orientation
        v_data=np.array(list(data.intensities))# Vector data
        ranges=np.array(list(data.ranges))
        ro=ranges[v_data>0.0]
        fi=self.angles[v_data>0.0]
        self.mask_raycasting=copy(self.exploration_FOV) # TODO: crear otra matriz que almacene el angulo de incidencia?
        cz=self.cell_size
        center=self.mask_raycasting.shape[0]//2
        #import pdb;pdb.set_trace()
        # Arma un mapa de ocupacion con los obtaculos pero sin el raycasting
        for idx,tita in enumerate(fi.tolist()):
            x=int(round((ro[idx]*np.cos(tita+orientation))/cz+center))
            y=int(round((ro[idx]*np.sin(tita+orientation))/cz+center))
            if x<center*2 and y<center*2 and x>0 and y>0:
                self.mask_raycasting[int(round(x)),int(round(y))]=False
        # interpola para evitar fallas del raycasting    
        MO=interpolacion(self.mask_raycasting)
                
        #plt.show()
        #import pdb;pdb.set_trace()
        # Implementa el raycasting
        self.get_ray_casting_mask2(~MO)
        #plt.figure(1)
        #plt.imshow(MO)
        #plt.figure(2)
        #plt.imshow(self.mask_raycasting)
        #plt.pause(1)
        #plt.close()
        
        #limits=self.mask_raycasting.shape
        #i=0            
        #for tita in fi.tolist():
        #    for k in range(int((self.FOV-ro[i])/cz)):
        #        for j in  range(2):
        #            # ang=tita+(j-0.5)*self.ang_step
        #            ang=tita#+(j)*self.ang_step
        #            ro_=ro[i]+cz*(k+1) # plus one because the first point is the obstacle and the others is unknown
        #            x=int((self.FOV+ro_*np.cos(ang+orientation))/cz)
        #            y=int((self.FOV+ro_*np.sin(ang+orientation))/cz)
        #            if x>=limits[0] or y>=limits[1] or x<0 or y<0:
        #                continue
        #            self.mask_raycasting[x,y]=False
        #    i+= 1                     
    def get_raycasting_mask3(self):
        data=self.data
        orientation=self.orientation
        v_data=np.array(list(data.intensities))# Vector data
        ranges=np.array(list(data.ranges))
        ro=ranges[v_data>0.0]
        fi=self.angles[v_data>0.0]
        self.mask_raycasting=copy(self.exploration_FOV) # TODO: crear otra matriz que almacene el angulo de incidencia?
        cz=self.cell_size
        center=self.mask_raycasting.shape[0]//2
        #import pdb;pdb.set_trace()
        # Arma un mapa de ocupacion con los obtaculos pero sin el raycasting
        r0=None
        umbral=cz*2
        delta_ang=np.math.asin(cz/self.FOV) 
        for idx,tita in enumerate(fi.tolist()):
            r_=np.linspace(ro[idx]+cz,self.FOV,int((self.FOV-ro[idx])*2/cz)) # crea el rayo "sombra" detras del obstaculo
            x=((r_*np.cos(tita+orientation))/cz+center).astype(int)
            y=((r_*np.sin(tita+orientation))/cz+center).astype(int)
            # Filtra los puntos que quedan fuera
            idx_x=x>0 ;  idx_y=y>0
            idx_=idx_x & idx_y
            idx_x=x<center*2 ;  idx_y=y<center*2
            idx_=idx_x & idx_y & idx_
            self.mask_raycasting[x[idx_], y[idx_]] = False
            if r0 is not None:
                if abs(r0-ro[idx])>umbral:  # Analisis de "continuidad del r"
                    # elimina una medicion que no es confiable
                    d=min(r0,ro[idx])
                    r_=np.linspace(d,self.FOV,int((self.FOV-d)*2/cz)) # crea el rayo "sombra" detras del obstaculo
                    x_1=np.round(((r_*np.cos(tita+orientation+delta_ang))/cz+center)).astype(int)
                    y_1=np.round(((r_*np.sin(tita+orientation+delta_ang))/cz+center)).astype(int)
                    x_2=np.round(((r_*np.cos(tita+orientation-delta_ang))/cz+center)).astype(int)
                    y_2=np.round(((r_*np.sin(tita+orientation-delta_ang))/cz+center)).astype(int)
                    x=np.hstack([x_1,x_2])
                    y=np.hstack([y_1,y_2])

                    idx_x=x>0 ;  idx_y=y>0
                    idx_=idx_x & idx_y
                    idx_x=x<center*2 ;  idx_y=y<center*2
                    idx_=idx_x & idx_y & idx_
                    self.mask_raycasting[x[idx_], y[idx_]] = False                                    

            r0=ro[idx]
           
        #import pdb;pdb.set_trace()
        #plt.imshow(self.mask_raycasting)
        #plt.show()    
    
    def get_raycasting_mask4(self):
        # TODO: si cambia la resolucion del laser, puede pasar que con un scan no cubra todo el mapa.
        # Calcula el mapa sdf y luego las normales
        sdf_map = distance_transform_edt(self.mask_raycasting)
        # los puntos que tengan obstaculos y que el angulo de insidencia diste del normal en 
        # mas de 45 grados, se ponen en False ya que no es confiable, salvo donde esta el obstaculo
        dx = convolve(sdf_map, np.array([[-1, 0, 1]]) / 2.0, mode='constant')
        dy = convolve(sdf_map, np.array([[-1], [0], [1]]) / 2.0, mode='constant')

        # Calcula las normales y normaliza
        normals = np.stack((dx, dy), axis=-1)
        # normaliza
        normals /= np.maximum(np.linalg.norm(normals, axis=-1, keepdims=True), 1e-6)  # Evita división por cero
        angles = np.arctan2(normals[:,:,1], normals[:,:,0])  
        # escaneo por angulo
        #plt.imshow(sdf_map)
        plt.imshow(angles)
        plt.show()
       
        test=np.zeros((fi.shape[0],2))
        for idx,tita in enumerate(fi.tolist()):
            rayo_ang=tita+orientation
            x=int((ro[idx]*np.cos(rayo_ang))/cz+center)
            y=int((ro[idx]*np.sin(rayo_ang))/cz+center)
            # Filtra los puntos que quedan fuera
            if x>0 and y>0 and x<center*2 and y<center*2:
  
                test[idx,0]=angles[x, y]
                test[idx,1]=rayo_ang
                #print(angles[x, y]-rayo_ang)
                #import pdb;pdb.set_trace()
                #self.mask_raycasting[x, y] = False
        
        import pdb;pdb.set_trace()
        
        
    def get_ray_casting_mask2(self,occupancy_map):
        """Sobre un mapa de ocupacion local, genera un mapa de raycasting teniendo en cuenta que el laser esta en el centro.
        Es necesario inicializar la clase con sus respectivos parametros.

        Args:
            occupancy_map (_type_): mapa de ocupacion local
        Return:
            devuelve una variable interna llamda ray_casting_mask.

        """
        # Implementa el ray casting mediante un mapa SDF. Deberia ser mas rapido
        # WARNING!!:  Esta todo en celdas, no en metros.
        # Generate SDF map using Fast Marching Method
        sdf_map = distance_transform_edt(~occupancy_map)
        #plt.imshow(sdf_map)
        #plt.show()
        map_size = sdf_map.shape[0]
        # LiDAR parameters
        fov = 2*np.pi  # Angular Field of view in radians
        resolution = self.ang_step  # Angular resolution in radians
        max_range = self.FOV/self.cell_size  -1 # Maximum range in cells
        c=(map_size//2)
        lidar_position = (c, c)

        # Convert fov and resolution to indices
        fov_indices = int(fov / resolution)
        half_fov_indices = fov_indices // 2

        # Initialize LiDAR occupancy map
        lidar_occupancy_map = np.zeros((map_size, map_size), dtype=bool)

        # Perform ray tracing using "bubbles"
        for i in range(fov_indices):
            angle = i * resolution - half_fov_indices * resolution
            r=0
            #import pdb;pdb.set_trace()
            while r<(max_range+1):  # Incremental step for ray casting
                x = int(lidar_position[0] + r * np.cos(angle))
                y = int(lidar_position[1] + r * np.sin(angle))

                if x < 0 or x >= map_size or y < 0 or y >= map_size: # se fija que no se salga del mapa
                    break
                
                distance_to_obstacle = sdf_map[x, y]
                #import pdb;pdb.set_trace()
                if distance_to_obstacle > 0:
                    lidar_occupancy_map[x, y] = True
                    if (distance_to_obstacle+r)>=max_range:
                        # si se pasa del rango maximo, pone todo el rayo blanco
                        x,y=self.set_angle_region(max_range,angle,lidar_position,sdf_map)   
                        # elimina los puntos que caen afuera del mapa
                        idx_x=x>0 ;  idx_y=y>0
                        idx=idx_x & idx_y
                        idx_x=x<map_size ;  idx_y=y<map_size
                        idx=idx_x & idx_y & idx

                        try:   
                            lidar_occupancy_map[x[idx], y[idx]] = True                                                           
                            #lidar_occupancy_map[x, y] = True                
                        except:
                            import pdb;pdb.set_trace()  
                        # poner todo el rayo blanco
                        break
                    else:
                        r=distance_to_obstacle+r

                if distance_to_obstacle ==0:
                    
                    x,y=self.set_angle_region(r,angle,lidar_position,sdf_map)                
                    lidar_occupancy_map[x, y] = True              
                    break
                
                #plt.imshow(lidar_occupancy_map)
                #plt.show()
                #import pdb;pdb.set_trace() 

        self.mask_raycasting=lidar_occupancy_map
        #self.mostrar_cosas(lidar_occupancy_map,occupancy_map,sdf_map)
    
    def build_LOS_map(self,occupancy_map,landmarks):
        """Desde el mapa de ocupacion total, genera un mapa "line of sight" que tiene True en los puntos desde donde se puede ver un landmark

        TODO: en construccion

        Args:
            occupancy_map (_type_): _description_
            landmarks (_type_): _description_

        Returns:
           LOS_map  (np.array): mapa booleano con True en los puntos desde donde se puede ver un landmark
        """
        sdf_map = distance_transform_edt(~occupancy_map.T)
        #plt.imshow(sdf_map)
        #indices = np.argwhere(occupancy_map)
        #import pdb;pdb.set_trace()
        #plt.plot(indices[:,0],indices[:,1],'ro')
        #plt.show()
        map_size = sdf_map.shape[0]
        # LiDAR parameters
        ang_= 2*np.pi  # Field of view in radians
        resolution = self.ang_step  # Angular resolution in radians
        max_range = (self.FOV-1.0)/self.cell_size  -1 # Maximum range in cells

        
        # Convert angles and resolution to indices
        ang_indices = int(ang_ / resolution)
        half_fov_indices = ang_indices // 2
        # Initialize LiDAR occupancy map
        LOS_map = np.full(shape=sdf_map.shape,fill_value=False)
        # LOS_map = np.zeros((map_size, map_size), dtype=bool)
        for landmark_position in landmarks:
            #landmark_position = (l[0], l[1])
            # Perform ray tracing using "bubbles"
            for i in range(ang_indices):
                angle = i * resolution #- half_fov_indices * resolution
                r=0
                while r<(max_range+1):  # Incremental step for ray casting
                    if abs(r)<7:
                        r=7 # los landmaks suelen ser obstaculos tambien, por esto se pone esta region de exclucion.
                    x = int(landmark_position[0] + r * np.cos(angle))
                    y = int(landmark_position[1] + r * np.sin(angle))

                    if x < 0 or x >= map_size or y < 0 or y >= map_size:
                        break
                    
                    distance_to_obstacle = sdf_map[x, y]
                    
                    #import pdb;pdb.set_trace()
                    if distance_to_obstacle > 0:
                        LOS_map[x, y] = True
                        if (distance_to_obstacle+r)>=max_range:
                            x,y=self.set_angle_region(max_range,angle,landmark_position)   
                            idx_x=x>0 
                            idx_y=y>0
                            idx=idx_x & idx_y
                            idx_x=x<map_size
                            idx_y=y<map_size
                            idx=idx_x & idx_y & idx

                            try:   
                                LOS_map[x[idx], y[idx]] = True                                                           
                                #lidar_occupancy_map[x, y] = True                
                            except:
                                import pdb;pdb.set_trace()  
                            # poner todo el rayo blanco
                            break
                        else:
                            r=distance_to_obstacle+r

                    if distance_to_obstacle <3: # Si pasa muy cerca de un obstaculo se toma como oclucion
                        x,y=self.set_angle_region(r,angle,landmark_position)                
                        LOS_map[x, y] = True    
                        #if angle>np.pi*3/2:
                        #    import pdb;pdb.set_trace()    
                        break

        return LOS_map.T

    def get_laser_data_for_obstacle_map(self,mask=np.array([[]])):
        # Crea un dato de laserscan a partir del mapa de obstaculos
        # dilation operator is applied to the mask to avoid the robot to be in the obstacle
        #TODO: very slow, improve the algorithm. 
        mask=self.dilation(mask)# cahnge False to True
        #mask=~mask
        self.data=lasermsg()
        self.data.angle_increment=self.ang_step
        self.data.angle_min=-np.pi
        self.data.angle_max=np.pi
        self.data.range_max=self.FOV
        self.data.range_min=0.0
        N=self.angles.shape[0]
        self.data.ranges=np.full(shape=[N],fill_value=np.Inf).tolist()
        self.data.intensities=np.full(shape=[N],fill_value=0.0).tolist()
        # Plot de la matriz booleana
        #plt.imshow(mask, cmap='gray')
        #plt.show()
        points=np.argwhere(mask) 
        #print(points.shape) # Son como 2mil puntos
        #import pdb;pdb.set_trace()
        for point in points:
            x=float(point[0])*self.cell_size-self.FOV
            #y=self.FOV-float(point[1])*self.cell_size
            y=float(point[1])*self.cell_size-self.FOV
            angle=np.arctan2(y,x)
            idx=np.argmin(abs(self.angles-angle))
            if self.data.ranges[idx]>np.sqrt(x**2+y**2):
                self.data.ranges[idx]=np.sqrt(x**2+y**2)
                self.data.intensities[idx]=1.0
                              
           
           # print("x: ",x," y: ",y," angle: ",angle," idx: ",idx)

        # debug only $1
        if self.debug:
            self.data.header.frame_id=rospy.get_name()[1:]
            self.data.header.stamp=rospy.Time.now()
            self.laser_pub.publish(self.data)  
            
    def dilation(self,mask=np.array([[]])):
        kernel = np.array([[0, 1, 0],
                       [1, 1, 1],
                       [0, 1, 0]])
        mask2 = binary_dilation((~mask).astype(float), kernel)
        mask2 = binary_dilation(mask2, kernel)
        return mask2.astype(bool)
    
    def create_laser_data(self):
        self.data=lasermsg()
        self.data.angle_increment=self.ang_step
        self.data.angle_min=-np.pi
        self.data.angle_max=np.pi
        self.data.range_max=self.FOV
        self.data.range_min=0.0
        N=self.angles.shape[0]
        self.data.ranges=np.full(shape=[N],fill_value=np.Inf).tolist()
        self.data.intensities=np.full(shape=[N],fill_value=0.0).tolist()
        return
    
    def get_laser_data_for_obstacle_map2(self,mask):
        # esta funcion hace lo mismo que la version 1, pero en lugar de hacerlo con los obstaculos, lo hace con los rayos del laser.
        mask=self.dilation(mask)# cahnge False to True
        self.create_laser_data()
        current_angle=-np.pi
        N=2*np.pi/self.ang_step
        self.angles
        for i in range(N):
            current_angle=self.ang_step+current_angle

    def set_angle_region(self,r,angle,lidar_position,sdf_map=None):
        """ Devuelve los puntos donde paso el rayo del lidar

        Args:
            r (_type_): _description_
            angle (_type_): _description_
            lidar_position (_type_): _description_

        Returns:
            _type_: _description_
        """
        
        r_=np.linspace(0,r,int(r))
        x = ((lidar_position[0] + r_ * np.cos(angle))).astype(int)
        y = ((lidar_position[1] + r_ * np.sin(angle))).astype(int)
        # elimina los puntos que pasan muy cerca de obstaculo. 
        if sdf_map is not None:

            idx=(sdf_map[x,y]<=2) & (sdf_map[x,y]>0)
            x=x[~idx]
            y=y[~idx]
            #import pdb;pdb.set_trace()
        return x,y


        
    def mostrar_cosas(self,lidar_occupancy_map,occupancy_map,sdf_map):
        # Display LiDAR occupancy map
        plt.figure(1,figsize=(8, 6))
        plt.imshow(lidar_occupancy_map, cmap='gray')#, origin='lower', extent=[0, map_size, 0, map_size])
        plt.title('LiDAR Occupancy Map')


        plt.figure(2,figsize=(10, 5))
        plt.subplot(1, 2, 1)
        plt.imshow(occupancy_map, cmap='gray')
        plt.title('Occupancy Map')
        plt.subplot(1, 2, 2)
        plt.imshow(sdf_map, cmap='jet')
        plt.colorbar(label='Distance')
        plt.title('Signed Distance Field (SDF) Map')
        plt.tight_layout()
        plt.show()


def interpolacion(MO):
        # MO=[11 12 ; 21 22]
        # MO=[1 2 ; 3 4] 
        mask_1=MO[:50,:50]
        mask_2=MO[:50,50:]
        mask_3=MO[50:,:50]
        mask_4=MO[50:,50:]
        MO_1=conv(mask_1)
        MO_2=conv(mask_2)
        MO_3=conv(mask_3)
        MO_4=conv(mask_4)
        mask_1[MO_1==18]=0
        mask_2[MO_2==14]=0
        mask_3[MO_3==12]=0
        mask_4[MO_4==8]=0
           # modifcacion
        mask_1[MO_1==14]=0
        mask_1[MO_1==12]=0
        mask_2[MO_2==18]=0
        mask_2[MO_2==8]=0
        mask_3[MO_3==18]=0
        mask_3[MO_3==8]=0
        mask_4[MO_4==14]=0
        mask_4[MO_4==12]=0
        MO_sup=np.hstack([mask_1,mask_2])
        MO_inf=np.hstack([mask_3,mask_4])
        MO_=np.vstack([MO_sup,MO_inf])
        return copy(MO_)
    
def conv(original_matrix, padding_value=0):
        # Matriz plantilla
        template_matrix = np.array([[0, 11, 0],
                                   [7, 0, 3],
                                   [0, 5, 0]])
        # Definir un kernel con la plantilla invertida
        kernel = np.flipud(np.fliplr(template_matrix))
        # Aplicar convolucion a la matriz original
        expanded_matrix = convolve2d(original_matrix, kernel, mode='same', boundary='symm', fillvalue=padding_value)
        # expanded_matrix = convolve2d(original_matrix, kernel, mode='same', boundary='fill', fillvalue=padding_value)
        return expanded_matrix
        
        
            
def main():
    rc=RayCasting()
    
    

if __name__=='__main__':
    main()    

