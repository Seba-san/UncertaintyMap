import paramiko
import pickle
import subprocess

class MyClass:
    def __init__(self, data):
        self.data = data

def run_on_remote_server():
    # Configura los detalles de la conexión SSH
    host = '190.124.224.160'
    username = 'idecom'
    password = '153246'  # O, si utilizas una clave pública, puedes omitir esto y configurar la clave SSH adecuadamente.
    import pdb; pdb.set_trace()
    # Crear una instancia de cliente SSH
    ssh_client = paramiko.SSHClient()

    # Ajusta la política de clave (en un entorno de producción, se debe usar una política más segura)
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # Conéctate al servidor
    ssh_client.connect(hostname=host, username=username, password=password)

  # Datos que deseas pasar con una estructura de clase
    my_object = MyClass(data='Hello from local!')

    # Serializar el objeto con pickle
    serialized_object = pickle.dumps(my_object)

    # Guardar los datos serializados en un archivo temporal
    with open('my_data.pkl', 'wb') as file:
        file.write(serialized_object)

    # Copiar el archivo con los datos serializados al servidor remoto usando scp
    subprocess.run(['scp', 'my_data.pkl', f'{username}@{host}:~/'])

    # Código que ejecutará el servidor para utilizar los datos
    command = f'ssh {username}@{host} "python -c \\"import pickle; from my_data.pkl import MyClass; with open(\'my_data.pkl\', \'rb\') as file: serialized_object = file.read(); my_object = pickle.loads(serialized_object); print(my_object.data)\\""'

    # Ejecutar el comando en el servidor remoto
    subprocess.run(command, shell=True)

    # Eliminar el archivo temporal local
    subprocess.run(['rm', 'my_data.pkl'])

if __name__ == '__main__':
    run_on_remote_server()