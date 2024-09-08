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


velocities = run[index]["velocities"]
for i in range(0, len(velocities), 3):
    # use red color
    dt = 0.4
    vx, vy = velocities[i]
    import numpy as np

    thr = footprints["orientations"][i]
    dx = dt * (np.cos(thr) * vx - np.sin(thr) * vy)
    dy = dt * (np.sin(thr) * vx + np.cos(thr) * vy)
    plt.arrow(
        footprints["positions"][i][0],
        footprints["positions"][i][1],
        dx,
        dy,
        head_width=0.05,
        head_length=0.05,
        fc="r",
        ec="r",
    )
px, py = zip(*footprints["positions"])
plt.plot(px, py, color="black", linewidth=2)

plt.plot(run[index]["start"][0], run[index]["start"][1], "yo")
plt.plot(run[index]["end"][0], run[index]["end"][1], "go")

obstacles = [[]] * len(footprints["obstacles"][0])
for i in range(len(footprints["obstacles"][0])):
    obstacles[i] = [p[i] for p in footprints["obstacles"]]
# plt.colorbar()
for obs in obstacles:
    # plt.scatter([p[0] for p in obs], [p[1] for p in obs], c=diffs, cmap=cm2, s=250)
    plt.scatter([p[0] for p in obs], [p[1] for p in obs], color="b", s=250)


# plt.colorbar()
plt.ylabel("Czas [s]")
plt.clim(0, 10)
plt.xlim(-9, 9)
plt.ylim(-9, 9)
plt.show()

# Create a colormap based on time
