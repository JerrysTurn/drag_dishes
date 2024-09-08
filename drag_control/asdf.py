import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Example list of 3D vectors, each vector is [x, y, z]
vectors = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]])

# Extracting x, y, and z coordinates for the points
X = vectors[:, 0]
Y = vectors[:, 1]
Z = vectors[:, 2]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
ax.scatter(X, Y, Z, color='b', s=50)  # 's' is the size of the points

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set the limits of the plot
ax.set_xlim([0, 15])
ax.set_ylim([0, 15])
ax.set_zlim([0, 15])

plt.title('3D Points in 3D Space')
plt.show()
