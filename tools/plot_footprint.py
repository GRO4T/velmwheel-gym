import pickle

import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

with open("footprint.pkl", "rb") as f:
    footprints = pickle.load(f)

# Plot the footprint
img = plt.imread("tools/2d_background.png")
plt.imshow(img, aspect="auto", extent=[-9, 9, -9, 9])
colors = [(0, 0, 1), (0, 1, 0), (1, 0, 0)]
colors2 = [(0, 1, 1), (0, 0, 0)]
diffs = [
    footprints["time"][i] - footprints["time"][0]
    for i in range(len(footprints["time"]))
]
cm = LinearSegmentedColormap.from_list("Custom", colors, N=256)
cm2 = LinearSegmentedColormap.from_list("Custom", colors2, N=256)
plt.scatter(
    [p[0] for p in footprints["position"]],
    [p[1] for p in footprints["position"]],
    c=diffs,
    cmap=cm,
    s=500,
)

obstacles = [[]] * len(footprints["obstacles"][0])
for i in range(len(footprints["obstacles"][0])):
    obstacles[i] = [p[i] for p in footprints["obstacles"]]
plt.colorbar()
for obs in obstacles:
    plt.scatter([p[0] for p in obs], [p[1] for p in obs], c=diffs, cmap=cm2, s=250)

# plt.colorbar()
plt.ylabel("Czas [s]")
plt.clim(0, 10)
plt.xlim(-9, 9)
plt.ylim(-9, 9)
plt.show()

# Create a colormap based on time
