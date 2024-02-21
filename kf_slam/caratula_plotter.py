import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy.io

# Cargar la matriz M desde el archivo de MATLAB
matlab_data = scipy.io.loadmat('matriz_linealizada.mat')
#matlab_data = scipy.io.loadmat('matriz_linealizada.mat')
M = matlab_data['M_']  # Asegúrate de que 'M' sea el nombre correcto en tu archivo

# Obtener las dimensiones de la matriz M
n = M.shape[0]

# Crear una malla de coordenadas para el gráfico
x = np.arange(n)
y = np.arange(n)
X, Y = np.meshgrid(x, y)

# Crear una figura y un eje en 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Crear el gráfico de superficie 3D
surf = ax.plot_surface(X, Y, M, cmap='jet')

# Configurar la barra de colores
fig.colorbar(surf)

# Mostrar el gráfico
plt.show()