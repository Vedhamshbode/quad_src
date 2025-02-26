import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_3d_points(points):
    # Extract x, y, z coordinates from the list of points
    x_vals = [point[0] for point in points]
    y_vals = [point[1] for point in points]
    z_vals = [point[2] for point in points]

    # Create a figure
    fig = plt.figure()
    
    # Add 3D axes
    ax = fig.add_subplot(111, projection='3d')

    # Scatter plot the points
    ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')

    # Set labels for the axes
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # Show the plot
    plt.show()

# Example usage
# List of 3D points
# Define the list to store points as lists
points = [
    [0.195, -0.054, 0.08],
    [0.235652, -0.054, 0.0662216],
    [0.280659, -0.054, 0.0533272],
    [0.279, -0.054, 0.04],
    [0.22943, -0.054, 0.0312148],
    [0.16057, -0.054, 0.0312148],
    [0.111, -0.054, 0.04],
    [0.109341, -0.054, 0.0533272],
    [0.154348, -0.054, 0.0662216],
    [0.195, -0.054, 0.08]
]





plot_3d_points(points)
