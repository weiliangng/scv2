import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create a grid of x and y values
x = np.linspace(1, 2.5, 100)
y = np.linspace(0, 15, 100)
X, Y = np.meshgrid(x, y)

# Calculate Z using the given equation
Z = 6.6667 * (0.15 * Y + X) - 16.6667

# Create the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
ax.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none')

# Labels
ax.set_xlabel('V(I_SET)')
ax.set_ylabel('I_LOAD')
ax.set_zlabel('I_BUFFER')

# Title
ax.set_title('3D Plot of z = 6.6667 * (0.15y + x) - 16.6667')

plt.show()
