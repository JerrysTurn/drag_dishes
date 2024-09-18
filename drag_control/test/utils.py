import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_rotation(theta):
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                [np.sin(theta),  np.cos(theta), 0],
                                [            0,              0, 1]])
    return rotation_matrix

def get_jacobian(x_r, y_r):
    
    jacobian_matrix = np.eye(3)
    jacobian_matrix[0, 2] = -y_r
    jacobian_matrix[1, 2] =  x_r

    return jacobian_matrix

def show_limit_surface(eigen_values):

    # Parameters for the ellipsoid
    a = eigen_values[0]  # Semi-axis length in x-direction
    b = eigen_values[1]  # Semi-axis length in y-direction
    c = eigen_values[2]  # Semi-axis length in z-direction
    
    # Parameters for the sphere
    radius = 1

    # Create a grid of points (u, v) for the ellipsoid
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    u, v = np.meshgrid(u, v)

    # Parametric equations for the ellipsoid
    x_ellipsoid = a * np.cos(u) * np.sin(v)
    y_ellipsoid = b * np.sin(u) * np.sin(v)
    z_ellipsoid = c * np.cos(v)

    # Parametric equations for the sphere
    x_sphere = radius * np.cos(u) * np.sin(v)
    y_sphere = radius * np.sin(u) * np.sin(v)
    z_sphere = radius * np.cos(v)

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the ellipsoid
    ax.plot_surface(x_ellipsoid, y_ellipsoid, z_ellipsoid, color='cyan', alpha=0.6, edgecolor='r', label='Ellipsoid')

    # Plot the sphere
    ax.plot_surface(x_sphere, y_sphere, z_sphere, color='blue', alpha=0.3, edgecolor='b', label='Sphere')

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set equal aspect ratio
    ax.set_box_aspect([a, b, c])  # Aspect ratio is set to match the semi-axes lengths

    # Show the plot
    plt.show()

def show_possible_velocity(velocity_candidate):
    # Extracting x, y, and z coordinates for the points
    X = velocity_candidate[:, 0]
    Y = velocity_candidate[:, 1]
    Z = velocity_candidate[:, 2]

    # Generate a list of colors using a colormap
    num_points = len(velocity_candidate)
    colors = plt.cm.viridis(np.linspace(0, 1, num_points))  # 'viridis' is an example colormap

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the points
    ax.scatter(X, Y, Z, color=colors, s=5)  # 's' is the size of the points

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set the limits of the plot
    ax.set_xlim([-0.3, 0.3])
    ax.set_ylim([-0.3, 0.3])
    ax.set_zlim([-0.3, 0.3])

    plt.title('3D Points in 3D Space')
    plt.show()