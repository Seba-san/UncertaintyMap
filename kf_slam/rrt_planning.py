import numpy as np
import random
import math
import matplotlib.pyplot as plt
from copy import deepcopy as copy

class Punto:
    """Crea un punto en el plano cartesiano
    """
    def __init__(self, x=0.0, y=0.0):

        self.p=np.array([x, y])
                
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = 0 # is a index of self.nodes list
        self.cost = np.NaN
        self.sigma_odo=0 # incertidumbre acumulada
        self.sigma_landmark=0.0 # No puede ser NaN xq no lo podes sumar si no viste un landmark en el camino...
        self.dist=0 # distancia total recorrida
    def __str__(self):
        return f'Node: x={self.x:.3f}, y={self.y:.3f}, parent={self.parent}, cost={self.cost:.3f}, sigma_odo={self.sigma_odo:.3f}, sigma_landmark={self.sigma_landmark:.3f}, dist={self.dist:.3f}'
    def __repr__(self):
        return f'Node: x={self.x:.3f}, y={self.y:.3f}, parent={self.parent}, cost={self.cost:.3f}, sigma_odo={self.sigma_odo:.3f}, sigma_landmark={self.sigma_landmark:.3f}, dist={self.dist:.3f}'


class RRTStar:
    """ Los puntos start, goal y el obstacle map estan dados en celdas."""
    def __init__(self, start: tuple, goal: tuple, obstacle_map, step_size: float, max_iter: int,nearest: int,
                 animation=False,landmarks=[],temperature=0,fov=30,LOS_map=np.array([])):
        self.start = Node(*start)
        self.start.sigma_odo=temperature
        self.start.cost=temperature
        self.goal = Node(*goal)
        #self.obstacle_list = obstacle_list
        self.obstacle_map=obstacle_map
        self.map_size=obstacle_map.shape
        (f,c)=np.where(obstacle_map)
        self.map_size_modifed=[min(f),min(c),max(f),max(c)]# [xmin,ymin,xmax,ymax]
        #import pdb;pdb.set_trace()
        
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.nearest=nearest
        self.show_animation = animation
        self.landmarks=landmarks # lista de landmarks en formato: [l1,l2,..], con l1=[x,y,sigma]
        self.deep_debug=False # Muesrta como se construye el arbol
        self.temperature=temperature# incertidumbre inicial del robot, si esta muy caliente va a elegir trayectorias que lo enfrien.
        self.fov=fov # field of view
        self.LOS_map=LOS_map # mapa de line of sight. Si es vacio no se usa.

    def generate_random_node(self):
        x_=random.uniform(0, self.map_size_modifed[3]-self.map_size_modifed[1])
        y_=random.uniform(0, self.map_size_modifed[2]-self.map_size_modifed[0])

        return Node(x_+self.map_size_modifed[1], y_+self.map_size_modifed[0])

    def find_nearest_node(self, random_node):
        """
         Busca el nodo mas cercano en terminos de distancia euclidiana. Devuelve el indice.
        """
        distances = [math.sqrt((random_node.x - node.x) ** 2 + (random_node.y - node.y) ** 2) for node in self.nodes]
        idx=distances.index(min(distances))
        #nearest_node = self.nodes[distances.index(min(distances))]
        return idx

    def steer(self, from_node, to_node):
        """
        Toma la direccion del random y genera un nuevo nodo en esa direccion
        """
        distance = math.sqrt((to_node.x - from_node.x) ** 2 + (to_node.y - from_node.y) ** 2)
        if distance < self.step_size:
            return to_node
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)
        return Node(new_x, new_y)

    def is_collision_free_old(self, from_node, to_node):
        """
        calcula la distancia punto a recta
        """        
        for (ox, oy, size) in self.obstacle_list:
            if distancia_punto_segmento(Punto(ox, oy), Punto(from_node.x, from_node.y), Punto(to_node.x, to_node.y))<=size:
                return False
        return True
    
    def is_collision_free(self, from_node, to_node):
        """
        se fija si el camino esta libre de obstaculos. 
        Para lograr esto se fija dentro de la region rectancular definida por los nodos from_node y to_node hay algun obstaculo,
        si lo hay devuelve False, si no hay devuelve True. El error que se comete al hacerlo asi es como maximo la distancia entre los nodos dividido 2.
        """      
        ax=round(min(from_node.x,to_node.x)-0.5)
        bx=round(max(from_node.x,to_node.x)+0.5) # para que no quede una linea de ancho cero.
        ay=round(min(from_node.y,to_node.y)-0.5)
        by=round(max(from_node.y,to_node.y)+0.5)              
       
        map_test=self.obstacle_map[ay:by,ax:bx] # estan dados vuelta los indices.
        #map_test=self.obstacle_map[ax:bx,ay:by] 
        if map_test.all():
            return True
        
        return False    

    def is_in_line_of_sight(self, from_node, to_node):   
        """Se fija si en el segmento que une from_node y to_node hay un lugar desde donde puede ver el obstaculo.

        Args:
            from_node (_type_): _description_
            to_node (_type_): _description_

        Returns:
            _type_: _description_
        """
        ax=round(min(from_node.x,to_node.x)-0.5)
        bx=round(max(from_node.x,to_node.x)+0.5) # para que no quede una linea de ancho cero.
        ay=round(min(from_node.y,to_node.y)-0.5)
        by=round(max(from_node.y,to_node.y)+0.5)              
       
        map_test=self.LOS_map[ay:by,ax:bx] # estan dados vuelta los indices.
        #map_test=self.obstacle_map[ax:bx,ay:by] 
        if map_test.all():
        #if map_test.any():
            return True
        
        return False 
 

    def plan(self):
        """
        metodo principal
        si encuentra un path valido devuelve un path en formato de lista por puntos como tuplas, es decir [(x1,y1),(x2,y2),...,(xn,yn)]
        """
        if self.show_animation: # TODO ver si se puede poner en otra funcion.
            fig, ax = plt.subplots()
            ax.axis("equal")
            ax.set_xlim(0, self.map_size[0])
            ax.set_ylim(0, self.map_size[1])                

            plt.imshow(self.obstacle_map)
            plt.plot(self.nodes[0].x, self.nodes[0].y, 'go', markersize=6)
            plt.plot(self.goal.x, self.goal.y, 'ro', markersize=6)

            theta = np.linspace(0, 2 * np.pi, 100)
            for l in self.landmarks:
                plt.plot(l[0], l[1], 'yx', markersize=6)
                x = l[0] + self.fov * np.cos(theta)
                y = l[1] + self.fov * np.sin(theta)
                plt.plot(x, y,'--y')


        for _ in range(self.max_iter):
            random_node = self.generate_random_node()
            idx = self.find_nearest_node(random_node)
            new_node = self.steer(self.nodes[idx], random_node)

            if self.is_collision_free(self.nodes[idx], new_node):
                # optimization
                new_node.parent = idx
                new_node=self.get_cost(self.nodes[idx], new_node)
                #new_node.cost = self.nodes[idx].cost + math.sqrt((new_node.x - self.nodes[idx].x) ** 2 + (new_node.y - self.nodes[idx].y) ** 2)
                min_cost_node,idx_ = self.find_min_cost_node(new_node)                                               
                                                      
                new_node=self.rewire(min_cost_node,idx_, new_node)  
                #new_node.parent=new__.parent
                #new_node.cost=new__.cost        
                
                self.nodes.append(new_node)
                
                if self.show_animation and self.deep_debug:                                                               
                    plt.plot([self.nodes[new_node.parent].x, new_node.x], [self.nodes[new_node.parent].y, new_node.y], 'g-')                                                                    
                    plt.plot(new_node.x, new_node.y, 'bo')
                    plt.draw()
                    plt.pause(0.01)      
                
                #if distancia_punto_segmento(Punto(self.goal.x, self.goal.y), Punto(new_node.x, new_node.y), Punto(self.nodes[new_node.parent].x, self.nodes[new_node.parent].y))<=self.step_size:
                #    p=proyeccion_punto_recta(Punto(self.goal.x, self.goal.y), Punto(new_node.x, new_node.y), Punto(self.nodes[new_node.parent].x, self.nodes[new_node.parent].y))          
                #    proy_node=Node(p[0],p[1])
                #    prev_node=self.nodes[new_node.parent]
                #    proy_node=self.get_cost(prev_node, proy_node)
                #    #proy_node.cost=prev_node.cost+math.sqrt((prev_node.x - proy_node.x) ** 2 + (prev_node.y - proy_node.y) ** 2)
                #    proy_node.parent=new_node.parent
                #    self.nodes.append(proy_node)
                #    self.goal.parent = len(self.nodes) - 1
                #    self.goal=self.get_cost(proy_node, self.goal) # no estoy seguro si es proy_node o prev_node
                #    #self.goal.cost = proy_node.cost + math.sqrt((self.goal.x - proy_node.x) ** 2 + (self.goal.y - proy_node.y) ** 2)                     
                #    return self.generate_final_path()  
        
        return self.is_connected((self.goal.x, self.goal.y))                                         

    def get_cost_(self, from_node: Node, to_node_: Node):
        d=math.sqrt((from_node.x - to_node_.x) ** 2 + (from_node.y - to_node_.y) ** 2)
        to_node=copy(to_node_)
        to_node.cost=d+from_node.cost
        return to_node
        
    def get_cost(self, from_node: Node, to_node_: Node):
        """Obtiene el costo entre dos nodos contemplando tanto la distancia euclidiana como la incertidumbre acumulada

        Args:
            from_node (Node): desde este nodo
            to_node (Node): hasta este nodo

        Returns:
           to_node (Node): to_node modificado con los nuevos parametros.
           
           Distancia calculada: d+d_odo+sigma_landmark. Distancia entre nodos, distancia recorrida con unicamente odometria e incertidumbre inicial (landmark)
        """
        to_node=copy(to_node_)
        
        d=math.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)

        bandera=0
        
        for idx,land in enumerate(self.landmarks): 
            #import pdb; pdb.set_trace()
            dist=distancia_punto_segmento(Punto(land[0],land[1]), Punto(to_node.x, to_node.y), Punto(from_node.x, from_node.y))

            if dist<self.fov and self.is_in_line_of_sight(from_node=from_node,to_node=to_node): # la trayectoria pasa por el radio de un landmark
                #import pdb; pdb.set_trace()
                if to_node.sigma_landmark==0:
                    if land[2]>to_node.sigma_odo:
                        self.landmarks[idx][2]=to_node.sigma_odo
                        land[2]=to_node.sigma_odo
                        to_node.sigma_landmark=land[2]
                    else:
                        to_node.sigma_landmark=land[2] # toma el sigma del landmark
                else:
                    #to_node.sigma_landmark=min(land[2]+to_node.sigma_odo,to_node.sigma_landmark) # toma el sigma del landmark
                    to_node.sigma_landmark=min(land[2],to_node.sigma_landmark) # toma el sigma del landmark $1 TODO, ojo aca.
                to_node.sigma_odo=0
                bandera=1
            
        if bandera==0:
            to_node.sigma_odo=from_node.sigma_odo+ d # acumula la incertidumbre de la trayectoria
            to_node.sigma_landmark=from_node.sigma_landmark
        
        to_node.dist=from_node.dist+d
        to_node.cost=to_node.dist+to_node.sigma_landmark+to_node.sigma_odo#+from_node.cost # el costo es la suma de la incertidumbre y la distancia recorrida

        # Se fija si el trayecto pasa por un landmark
        # si es asi, toma el menor sigma entre el sigma que tiene la trayectoria y el sigma del landmark
        # si gana la trajectoria, el landmark toma el valor de la trayectoria, sino la trayectoria toma el valor del landmark
        # si no pasa por un landmark, el sigma es el de la trayectoria
        # el sigma de la trayectoria es proporcional a la distancia recorrida

        return to_node

    def find_min_cost_node(self, node: Node):
        """
            Devuelve el nodo con el menor costo entre los nodos que estan a una distancia menor a self.nearest
            Devuelve el nodo mas cercano y el indice correspondiente a self.nodes
            Ademas garantiza que no haya obstaculos entre el nodo y el punto con menor costo.
        """
        # crea la lista de nodos cercanos
        nearby_nodes = []  #
        for idx,n in enumerate(self.nodes):            
            distance = math.sqrt((n.x - node.x) ** 2 + (n.y - node.y) ** 2)
            if distance <= self.nearest:
                if self.is_collision_free(n, node):
                    nearby_nodes.append([n,idx])  # agrega a la lista todos los nodos que estan a una distancia menor a 2*step_size
        # Selecciona el nodo con el menor costo de la lista nearby_nodes
        if not nearby_nodes: 
            print('The Node is not connected to the tree')
            return None,None
        bandera=0  
        #import pdb;pdb.set_trace()      
        for n in nearby_nodes:
            if bandera==0:
                min_cost_node=n[0]
                idx=n[1]
                bandera=1
                continue
            if n[0].cost<min_cost_node.cost: #  TODO: me parece que hay que calcular el costo de n al node, no simplemente comparar costos.
                min_cost_node=n[0]
                idx=n[1]
        return min_cost_node,idx  

    def rewire(self, min_cost_node:Node,idx_:int,new_node: Node):
        """Revisa si el costo del camino desde new_node es menor que el costo del camino desde min_cost_node. 
            No entiendo xq lo hice asi..

        Args:
            min_cost_node (Node): _description_
            idx_ (int): _description_
            new_node (Node): _description_

        Returns:
            _type_: _description_
        """
        #import pdb;pdb.set_trace()
        #new_=Node(new_node.x,new_node.y) 
        node_=self.get_cost(min_cost_node, new_node)
        if node_.cost < new_node.cost:
            node_.parent=idx_
            return node_
        return new_node
    
        if min_cost_node.cost + math.sqrt((new_node.x - min_cost_node.x) ** 2 + (new_node.y - min_cost_node.y) ** 2) <= new_node.cost:
            new_.parent=idx_
            new_.cost=min_cost_node.cost + math.sqrt((new_node.x - min_cost_node.x) ** 2 + (new_node.y - min_cost_node.y) ** 2)
            return new_
        
        return new_node

    def generate_final_path(self):
        
        self.replanning()
        path = []
        current_node = self.goal
        bandera=1
        while True:            
            path.append((current_node.x, current_node.y))
            #print(current_node.cost)
            if current_node.parent==0 and bandera==0:
                break
            else:
                if current_node.parent==0:
                    bandera=0
                current_node = self.nodes[current_node.parent]
                    
        return path[::-1]
    
    def is_connected(self,point: tuple):
        """ Check if the point is connected to the tree. 
            the start node is the root of the tree. It doesn't change during the process 'cause 
            needs to recompute all cost again.
        """
        node_query = Node(*point)
        if self.show_animation:
            fig, ax = plt.subplots()
            ax.axis("equal")
            ax.set_xlim(0, self.map_size[0])
            ax.set_ylim(0, self.map_size[1])                

            plt.imshow(self.obstacle_map)
            plt.plot(self.nodes[0].x, self.nodes[0].y, 'go', markersize=6)
            plt.plot(node_query.x, node_query.y, 'ro', markersize=6)
        
        n,idx = self.find_min_cost_node(node_query)
        if n is None:
            return None
        # genera la ruta desde el nodo mas cercano al punto hasta el punto
        self.goal=Node(*point)
        self.goal.parent = idx
        self.goal=self.get_cost(n, self.goal) 
        
        #self.goal.cost = n.cost + math.sqrt((self.goal.x - n.x) ** 2 + (self.goal.y - n.y) ** 2)                     
        return self.generate_final_path()
    
    def replanning(self):
        """Una vez que se encuentra un camino, se puede hacer replanning para mejorarlo. 
        La idea es que por cada nodo del path encontrado, busque en una cercania si hay un camino mas corto. Si es asi cambia el aperent y  el costo y vuelve a buscar.
        """
        current_node=self.goal
        #import pdb;pdb.set_trace()
        iterations=100
        avoid_loops=[]
        while iterations>0:
            iterations-=1
            min_cost_node,idx_ = self.find_min_cost_node(current_node) 
            current_node_=self.rewire(min_cost_node,idx_, current_node) # revisar TODO. El rewire tendria que ser inverso xq vas de atras hacia adelante.
            if current_node_.parent==0 or current_node.parent==0: # llego al nodo inicial
                break 
            
            if current_node_.parent in avoid_loops:
                avoid_loops.append(current_node.parent)
                current_node=self.nodes[current_node.parent]
            else:
                current_node=self.nodes[current_node_.parent]                
                avoid_loops.append(current_node_.parent)
        
        if iterations==0:
            # Salio porque entro en un bucle
            import pdb;pdb.set_trace()
        return
            
def proyeccion_punto_recta(p=Punto(), a=Punto(), b=Punto()):
    v=b.p-a.p
    u=p.p-a.p
    proy=v*np.dot(u,v)/np.dot(v,v)    
    p_final=a.p+proy    
    return p_final

def distancia_punto_segmento(p: Punto,a: Punto,b: Punto)->float:
    """Devuele la distancia ortogonal entre un punto y un segmento definido entre dos puntos

    Args:
        p (Punto): Punto a calcular la distancia
        a (Punto): Punto del segmento
        b (Punto): Punto del segmento

    Returns:
        float: distancia
    """
    v=b.p-a.p
    u_a=p.p-a.p
    u_b=p.p-b.p
    d_segment=np.sqrt(np.dot(v,v))
    if d_segment<0.1: # TODO: revisar
        d_segment=0.1
        #return np.sqrt(np.dot(u_a,u_a))
    #print(d_segment)
    try:
        K=v/(np.power(d_segment,2))
    except:
        print('Ocurrio un problema, revisar. d_sgment: ',d_segment)
        import pdb; pdb.set_trace()
        
    proy_a=K*np.dot(u_a,v)
    proy_b=K*np.dot(u_b,v)
    err=0.1# errores de redondeo y demas.
    if np.sqrt(np.dot(proy_a,proy_a))+np.sqrt(np.dot(proy_b,proy_b))<=d_segment+err:
        # dentro del segmento, distancia perpendicular
        perpedicular=u_a-proy_a
        distancia=np.sqrt(np.dot(perpedicular,perpedicular))
        return distancia
    else:
        # fuera del segmento, distancia al punto mas cercano
        distancia=min(np.sqrt(np.dot(u_a,u_a)),np.sqrt(np.dot(u_b,u_b)))
        return distancia  

if __name__ == '__main__':
    
    # Ejemplo de uso
    #start = (10, 10)
    
    #obstacle_list = [(40, 10, 10),(40, 30, 10), (40, 50, 10),(40, 90, 10),(60, 50, 20),(60, 80, 15),(70, 30, 10),(150, 100, 50),(230, 70, 20)]
    obstacle_map=np.ones((200,200))
    start=(50,100)
    goal = (100,100)
    temperature=50
    landmarks=[[150,150,50]]

    rrt_star = RRTStar(start, goal, obstacle_map, step_size=5, max_iter=300,nearest=100,animation=True,landmarks=landmarks,temperature=temperature,fov=40)
    rrt_star.deep_debug=True
    path = rrt_star.plan()

    if path:
        print("Camino encontrado:")
        print(path)
        print("Costo")
        print(rrt_star.nodes[-1].cost)

    else:
        print("No se pudo encontrar un camino.")

    if path:
        path = np.array(path)
        plt.plot(path[:,0], path[:,1], 'r--')
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()

    plt.show()