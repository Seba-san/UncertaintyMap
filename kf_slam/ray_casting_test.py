import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
import time

def set_angle_region(r,angle,lidar_position):
    r_=np.linspace(0,r,r)
    x = (lidar_position[0] + r_ * np.cos(angle)).astype(int)
    y = (lidar_position[1] + r_ * np.sin(angle)).astype(int)
    return x,y

def dibujar_cuadrado(mapa, cx, cy, lado):
    for i in range(cy, cy + lado):
        for j in range(cx, cx + lado):
            mapa[i][j] = True

    return mapa
# Create an empty occupancy map
map_size = 300
occupancy_map = np.zeros((map_size, map_size), dtype=bool)

# Create a circle in the occupancy map
circle_radius = 10
circle_center = (map_size // 2, map_size // 2)
for y in range(map_size):
    for x in range(map_size):
        if np.sqrt((x - circle_center[0])**2 + (y - circle_center[1])**2) <= circle_radius:
            occupancy_map[y, x] = True


occupancy_map=dibujar_cuadrado(occupancy_map, 50, 100, 30)


# Generate SDF map using Fast Marching Method
sdf_map = distance_transform_edt(~occupancy_map)

# LiDAR parameters
fov = 360  # Field of view in degrees
resolution = 1  # Angular resolution in degrees
max_range = 100  # Maximum range in meters
lidar_position = (150, 100)

time1=time.time()
# Convert fov and resolution to indices
fov_indices = int(fov / resolution)
half_fov_indices = fov_indices // 2

# Initialize LiDAR occupancy map
lidar_occupancy_map = np.zeros((map_size, map_size), dtype=bool)

# Perform ray tracing using "bubbles"
for i in range(fov_indices):
    angle = np.radians(i * resolution - half_fov_indices * resolution)
    r=0
    #import pdb;pdb.set_trace()
    while r<(max_range+1):  # Incremental step for ray casting
        x = int(lidar_position[0] + r * np.cos(angle))
        y = int(lidar_position[1] + r * np.sin(angle))
        
        if x < 0 or x >= map_size or y < 0 or y >= map_size:
            break
        
        distance_to_obstacle = sdf_map[x, y]
        #import pdb;pdb.set_trace()
        if distance_to_obstacle > 0:
            lidar_occupancy_map[x, y] = True
            if (distance_to_obstacle+r)>=max_range:
                
                x,y=set_angle_region(max_range,angle,lidar_position)                
                lidar_occupancy_map[x, y] = True                
                # poner todo el rayo blanco
                break
            else:
                r=distance_to_obstacle+r
            
        if distance_to_obstacle ==0:
            
            x,y=set_angle_region(r,angle,lidar_position)                
            lidar_occupancy_map[x, y] = True              

            break




# Generar región circular
#circular_region = generate_circular_region(width, height, center, radius, angle_opening)

# Establecer valores en el mapa
#mapa[circular_region] = True



print(time.time()-time1)
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


