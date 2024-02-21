"""La idea de este script es probar algunas cosas del algoritmo raycasting implementado
"""

from raycasting import RayCasting
from sensor_msgs.msg import LaserScan 
from matplotlib import pyplot as plt
from copy import deepcopy as copy
import numpy as np
from scipy.signal import convolve2d


def create_figure():
    x=np.linspace(0,5,50)
    y=[]
    pendiente=-1.0
    for x_ in x:
        y.append(pendiente*x_+2.5)
    
    y=np.array(y)
    
    
    angulos_puntos=np.arctan2(y,x)
    
    angles=np.linspace(-np.pi,np.pi,int(1+np.pi*2/0.008738775737583637))
    ang_step=0.008738775737583637 # radians
    
    
    
    ranges=[]
    intensities=[]
    for idx,ang in enumerate(angles):
        if ang<np.pi/2 and ang>0:
            idx_=np.argmin(abs(ang-angulos_puntos))
            ranges.append(np.sqrt(y[idx_]**2+x[idx_]**2))
            intensities.append(1.0)
        else:
            intensities.append(0.0)
            ranges.append(0.0)

    
    
    #plt.plot(x,y,'.b')
    #plt.show()
    return ranges,intensities
    
def make_mask_FOV():
        """ make a mask of field of view to generate map of exploration
        """
        r=5.0
        cz=0.1
        a=int(2*r/cz); b=a
        exploration_FOV=np.full(shape=[a,b],fill_value=0)
        
       
        for i in range(a):
            for k in range(b):
                x=float(i)*cz
                y=float(k)*cz
                if (x-r)**2+(y-r)**2<(r-cz)**2:
                    exploration_FOV[i,k]=1
                    
        return exploration_FOV
    
def create_figure_for_obstacle_map(MO,pendiente=1):
    # lineas paralelas en sentido 2 - 7  (del reloj) (con pendiente=-1)
    # lineeas paralelas en sentido 10 - 5 (del reloj) (con pendiente=1)
    # Linea 1
    x=np.linspace(-4.0,4.0,160)
    y=[]
    #pendiente=-1
    ordenada=-2
    for x_ in x:
        y.append(pendiente*x_+ordenada)
    
    y=np.array(y)
    for x_,y_ in zip(x,y):
        k=int(round(y_*10))+50
        j=int(round(x_*10))+50
        if k<100 and j<100 and k>0 and j>0:
           MO[k,j]=0
    # Linea 2
    x=np.linspace(-4.0,4.0,160)
    y=[]
    #pendiente=-1
    ordenada=2
    for x_ in x:
        y.append(pendiente*x_+ordenada)
    
    y=np.array(y)
    for x_,y_ in zip(x,y):
        k=int(round(y_*10))+50
        j=int(round(x_*10))+50
        if k<100 and j<100 and k>0 and j>0:
           MO[k,j]=0
    return copy(MO)

def insert_circle(MO,r=2,cx=40,cy=80):
    # incerta un circulo donde cx es el sentido vertical y cy el horizonal (se muestra con un imshow, x eso estan invertidos)
    #r=2.0
    cz=0.1
    r_=r/cz
    n=720
    angulo_step=2*np.pi/n         
    #cx=40
    #cy=80
    for i in range(n):
        x=cx+(r_)*np.math.cos(i*angulo_step)
        y=cy+(r_)*np.math.sin(i*angulo_step)
        if x+1<100 and y+1<100 and x>0 and y>0:
            try:
                MO[int(round(x)),int(round(y))]=0
            except:
                import pdb;pdb.set_trace()
        
    #MO[50:50+a,50:50+b]=exploration_FOV
    return copy(MO)

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

if __name__=='__main__':
    # crea los puntos
    ranges,intensities= create_figure()
    # crea un mapa de ocupacion
    MO=make_mask_FOV()

    MO_=create_figure_for_obstacle_map(MO,pendiente=1)
    # MO_=insert_circle(MO,r=2,cx=50,cy=80) 
    plt.figure(1)
    plt.imshow(MO_)
    plt.title('puntos laser')
    
    MO_=interpolacion(MO_)
    #import pdb;pdb.set_trace()
    plt.figure(2)
    plt.imshow(MO_)
    plt.title('interpolacion')
    #plt.show()
    # hay que revisar que pasa para distintos angulos. Hay que poner el valor minimo del sdf=1 (en lugar de 0) para que no tenga filtraciones
    
    RC=RayCasting()
    #data=LaserScan()
    #data.intensities=intensities
    #data.ranges=ranges
    #RC.data=copy(data)
    
    # obtiene la mascara
    #RC.get_raycasting_mask()
    RC.get_ray_casting_mask2(~MO_)
    mask=RC.mask_raycasting
    # muestra resultados
    plt.figure(3)
    plt.imshow(mask)#,origin='lower')
    plt.title('ocupacion')
    plt.show()
    