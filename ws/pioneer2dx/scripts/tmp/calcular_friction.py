import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import ApplyBodyWrench
import time
import csv
class Pose:
    def __init__(self,name):
        self.name=name
        #self.sub=rospy.Subscriber(name,PoseStamped,self.callback)
        #rospy.spin()

    def callback(self):
        #data=PoseStamped()
        data=rospy.wait_for_message(self.name,PoseStamped)
        self.x=data.pose.position.x
        self.y=data.pose.position.y
        return self.x

class Force:
    def __init__(self,name):
        self.name=name
        self.apply_force=rospy.ServiceProxy(self.name,ApplyBodyWrench)
        self.get_msg()
    
    def get_msg(self):
        msg=ApplyBodyWrench._request_class()
        msg.body_name='robot1::base_link'
        msg.reference_frame='robot1::base_link'
        msg.reference_point.x=0.1
        #msg.reference_point.z=-0.037 # eje de las ruedas
        msg.reference_point.z=-0.145 # Directo al piso
        msg.duration.secs=10.0
        #msg.wrench.force.x=20
        self.msg=msg

    def call(self):
        rospy.wait_for_service(self.name)
        self.apply_force(self.msg)

def guardar_data(D):
    print(D)
    with open('dict.csv', 'w') as csv_file:  
        writer = csv.writer(csv_file)
        writer.writerow(D['Fuerza_aplicada'])
        writer.writerow(D['Fuerza_neta'])
       # for key, value in D.items():
       #    writer.writerow([key, value])

def calcular_aceleracion(dx,fza):
    print('El desplazamiento fue de {}'.format(dx))
    a=2*dx/100
    fn=12*a
    print('La fuerza neta fue de {}'.format(fn))
    mu=(fza-fn)/(12*9.8)
    print('mu equivalente {}'.format(mu))
    return fn

def main():
    D={'Fuerza_aplicada':[],'Fuerza_neta':[]}
    rospy.init_node('calc_force')
    vehiculo=Pose('/robot1/pioneer2dx/ground_truth/pose')
    force=Force('/gazebo/apply_body_wrench')
    print('Iniciando')
    loop=True
    force.msg.wrench.force.x=0
    force.msg.wrench.force.y=0
    while loop:
        force.msg.wrench.force.x+=1
        #force.msg.wrench.force.y+=5
        print('Fuerza aplicada {}'.format(force.msg.wrench.force.x))
        #print('Fuerza aplicada {}'.format(force.msg.wrench.force.y))
        x1=vehiculo.callback()
        force.call()
        time.sleep(10)
        x2=vehiculo.callback()
        fn=calcular_aceleracion(x2-x1,force.msg.wrench.force.x)
        D['Fuerza_aplicada'].append(force.msg.wrench.force.x)
        D['Fuerza_neta'].append(x2-x1)
        #guardar_data(D)

        if abs(x2)>20:
            loop=False
        else:
            time.sleep(5)

    guardar_data(D)



if __name__=='__main__':
    main()
