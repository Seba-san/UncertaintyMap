import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

# Generamos algunos datos de ejemplo
np.random.seed(0)
X = np.random.randn(300, 2)

# Definimos el número de clusters
k = 3

# Creamos una instancia de KMeans y lo ajustamos a nuestros datos
kmeans = KMeans(n_clusters=k)
kmeans.fit(X)

# Obtenemos los centros de los clusters y las etiquetas para cada punto
centers = kmeans.cluster_centers_
labels = kmeans.labels_

# Graficamos los datos y los centros de los clusters
fig, ax = plt.subplots()
colors = ["r", "g", "b"]
for i in range(k):
    ax.scatter(X[labels == i][:,0], X[labels == i][:,1], c=colors[i], label="Cluster "+str(i+1))
ax.scatter(centers[:,0], centers[:,1], c="k", marker="x", s=200, label="Centers")
ax.legend()
plt.show()