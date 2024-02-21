import re
import matplotlib.pyplot as plt
import numpy as np
import sys


def main(data):

    waypoints = []
    divergences = []

    pattern = r"control signals:\s(.*?)\ndivergencia fast:\s+(\d+\.\d+)"

    #pattern = r"divergencia:\s+(\d+\.\d+)"
    matches = re.findall(pattern, data, re.DOTALL)
    #import pdb; pdb.set_trace()
    for match in matches:
        waypoint = eval(match[0])
        divergence = float(match[1])
        waypoints.append(waypoint)
        divergences.append(divergence)

    #import pdb; pdb.set_trace()
    divergences = np.array(divergences)
    colors = plt.cm.jet((divergences - np.min(divergences)) / (np.max(divergences) - np.min(divergences)))

    fig, ax = plt.subplots()

    for i, waypoint in enumerate(waypoints):
        ax.scatter(-waypoint[0][0], waypoint[0][1], c=[colors[i]])#, label=f"Divergence: {divergences[i]:.2f}")

    ax.set_xlabel('Control Signal X')
    ax.set_ylabel('Control Signal Y')
    ax.set_title('Control Signals and Divergences')
    ax.legend()
    #cbar = fig.colorbar(sc, ax=ax)
    #cbar.set_label('Divergence')
    ax.axis('equal')  # Igualar los ejes
    plt.show()

if __name__ == "__main__":
#def __name__=='__main__':
    data = """
control signals:  [[0, 0], [0, 0]]
divergencia fast:  1359.313977326347
Colision detectada
control signals:  [[0.0, -0.3076923076923075], [0, 0]]
divergencia fast:  1361.191595895566
Colision detectada
control signals:  [[0.3076923076923077, -0.3076923076923075], [0, 0]]
divergencia fast:  1359.5956201117297
Colision detectada
control signals:  [[2.4615384615384617, 2.46153846153846], [0, 0]]
divergencia fast:  1506.767746279607
Colision detectada
control signals:  [[0.0, 3.384615384615382], [0, 0]]
divergencia fast:  1454.5089465107355
Colision detectada
control signals:  [[3.0769230769230766, 0.0], [0, 0]]
divergencia fast:  1503.4295279098649
Colision detectada
control signals:  [[-2.4615384615384617, 2.46153846153846], [0, 0]]
divergencia fast:  1460.5598639120888
Colision detectada
control signals:  [[-3.384615384615384, 0.0], [0, 0]]
divergencia fast:  1487.2909543809167
Colision detectada
control signals:  [[-2.4615384615384617, -2.461538461538461], [0, 0]]
divergencia fast:  1531.4612976003857
"""
    # main(sys.argv[1])
    main(data)
