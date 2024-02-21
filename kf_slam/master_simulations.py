import subprocess
import time
#import psutil
import sys
import ruamel.yaml
import datetime

def run_command(command,output=True):
    if output:          
        process = subprocess.Popen(command, shell=True)   
    else:
        #process = subprocess.Popen(command, shell=True,stdout=null_device) 
        pass  
        
    return process

def update_yaml_file(file_path, new_x, new_y,planner,mpc=False,UF=False,sigma_max=0.6,map='nada'):
    yaml = ruamel.yaml.YAML()

    # Leer el contenido del archivo YAML
    with open(file_path, 'r') as file:
        data = yaml.load(file)

    # Modificar los valores de initial_pose.x e initial_pose.y
    data['initial_pose']['x'] = new_x
    data['initial_pose']['y'] = new_y
    data['planner'] = planner
    data['planner_mpc']=mpc
    data['UF']=UF
    data['sigma_max']=sigma_max
    data['map']=map


    # Escribir los cambios en el archivo YAML preservando la estructura original
    with open(file_path, 'w') as file:
        yaml.dump(data, file)

def read_yaml_file(file_path):
    yaml = ruamel.yaml.YAML()
    with open(file_path, 'r') as file:
        data = yaml.load(file)
    return data

if __name__=='__main__':
    # leer la primera linea del archivo de simulaciones planificadas
    s_p=read_yaml_file('simulaciones_planificadas.yaml')
    for i in range(len(s_p)):
        update_yaml_file('parameters.yaml',s_p[i]['x'],s_p[i]['y'],s_p[i]['planner'],s_p[i]['mpc'],s_p[i]['UF'],s_p[i]['sigma_max'],s_p[i]['map'])
        timestamp = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
        name='./matlab/'+timestamp+'_simulacion_'+str(i)

        #Tarea 1: Ejecutar prepare_simulation.py
        print('##################### PREPARE SIMULATION  1/6 ##################### N: ' +str(i))
        p=run_command("python prepare_simulation.py")
        p.wait()
        print('##################### LAUNCH SIMULATION  2/6 ##################### N: ' +str(i))

        # Tarea 2: Ejecutar roslaunch test.launch
        ros_process = run_command("roslaunch test.launch")

        # Esperar 10 segundos
        time.sleep(20)
        print('##################### LAUNCH ACTIVE SLAM 3/6 ##################### N: ' +str(i))
        # Tarea 3: Ejecutar python active_slam_core.py en un proceso paralelo
        # active_slam_process = run_command("python active_slam_core.py")
        waypoint = run_command("python way_point_manager.py")
        active_slam_process = run_command("python active_slam_core_rrt.py")
        #active_slam_process = run_command("python active_slam_core_test.py")
        # Esperar a que el proceso active_slam_core.py termine
        active_slam_process.wait(timeout=2500)
        waypoint.terminate()
        #active_slam_process.terminate()
        print('##################### ACTIVE SLAM FINALIZED 4/6 ##################### N: ' +str(i))

        # Tarea 4: Ejecutar python save_map.py
        p=run_command("python save_map.py "+name)
        p.wait(timeout=120)
        p.terminate()
        print('##################### DATA SAVED 5/6 ##################### N: ' +str(i))
        # Esperar a que el proceso save_map.py termine
        p=subprocess.call(['rosnode', 'kill', '--all'])
        print('##################### SIMULATION CLOSED 6/6 ##################### N: ' +str(i))
        time.sleep(60)
