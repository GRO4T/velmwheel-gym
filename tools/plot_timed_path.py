import pickle

import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

with open("run.pkl", "rb") as f:
    run = pickle.load(f)

index = 1

footprints = run[index]["footprints"]
# Plot the footprint
img = plt.imread("tools/2d_background.png")
plt.imshow(img, aspect="auto", extent=[-9, 9, -9, 9])

# Colormap
colors = [(0, 0, 1), (0, 1, 0), (1, 0, 0)]
colors2 = [(0, 1, 1), (0, 0, 0)]
diffs = [
    footprints["time"][i] - footprints["time"][0]
    for i in range(len(footprints["time"]))
]
cm = LinearSegmentedColormap.from_list("Custom", colors, N=256)
cm2 = LinearSegmentedColormap.from_list("Custom", colors2, N=256)
plt.scatter(
    [p[0] for p in footprints["positions"]],
    [p[1] for p in footprints["positions"]],
    c=diffs,
    cmap=cm,
    s=500,
)

obstacles = [[]] * len(footprints["obstacles"][0])
for i in range(len(footprints["obstacles"][0])):
    obstacles[i] = [p[i] for p in footprints["obstacles"]]
plt.colorbar()
for obs in obstacles:
    # plt.scatter([p[0] for p in obs], [p[1] for p in obs], c=diffs, cmap=cm2, s=250)
    plt.scatter([p[0] for p in obs], [p[1] for p in obs], color="b", s=250)


plt.plot(run[index]["start"][0], run[index]["start"][1], "yo")
plt.plot(run[index]["end"][0], run[index]["end"][1], "go")


# plt.colorbar()
plt.ylabel("Czas [s]")
plt.clim(0, 10)
plt.xlim(-9, 9)
plt.ylim(-9, 9)
plt.show()

# Create a colormap based on time
