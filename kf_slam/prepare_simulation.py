import yaml
import xml.etree.ElementTree as ET

def read_initial_pose_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        parameters = yaml.safe_load(file)
        initial_pose = parameters['initial_pose']
        x = float(initial_pose['x'])
        y = float(initial_pose['y'])
        #if  ~isinstance(x, float):
        #    print('x no es un float')
        #if  ~isinstance(y, float):
        #    print('y no es un float')
        return x, y

def set_init_pose(xml_file, x=0.0, y=0.0, z=0.0):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    for include_elem in root.iter('include'):
        for arg_elem in include_elem.iter('arg'):
            if arg_elem.get('name') == 'init_pose':
                arg_elem.set('value', f'-x {x} -y {y} -z {z}')

    tree.write(xml_file)


if __name__ == '__main__':
    # Uso del programa de ejemplo
    yaml_file = 'parameters.yaml'  # Ruta al archivo YAML
    xml_file = '../../launch/multiple_robots.launch'  # Ruta al archivo XML

    x, y = read_initial_pose_from_yaml(yaml_file)

    # Valores adicionales para la pose
    z = 0.5  # Valor de la pose en z

    set_init_pose(xml_file, x, y, z)
    # Modifico los 2 xq no se cual se usa. Depende del caso. TODO: ver como hacer para que se modifique solo el que se usa.
    xml_file = 'multiple_robots.launch'  # Ruta al archivo XML
    set_init_pose(xml_file, x, y, z)
    print('Pose setted in x=',x,' y=',y,' z=',z)