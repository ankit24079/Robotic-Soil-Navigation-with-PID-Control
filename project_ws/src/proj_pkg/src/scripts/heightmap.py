import numpy as np
import matplotlib.pyplot as plt
from noise import pnoise2

width = 100
height = 100
scale = 10.0

heightmap = np.zeros((height, width))
for y in range(height):
    for x in range(width):
        heightmap[y][x] = pnoise2(x /scale, y / scale)

heightmap = np.interp(heightmap, (heightmap.min(), heightmap.max()), (0, 255))

plt.imsave("heightmap.png", heightmap, cmap="Greens")