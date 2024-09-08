import pickle

import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

with open("run.pkl", "rb") as f:
    runs = pickle.load(f)

index = 1

# footprints = run[index]["footprints"]
# Plot the footprint
img = plt.imread("tools/2d_background.png")
plt.imshow(img, aspect="auto", extent=[-9, 9, -9, 9])

# Draw arrow from start to end
for run in runs:
    start = run["start"]
    end = run["end"]

    translation = 0.05
    if end[1] > start[1]:
        start = (start[0] + translation, start[1])
        end = (end[0] + translation, end[1])
    else:
        start = (start[0] - translation, start[1])
        end = (end[0] - translation, end[1])

    pos_x, pos_y = zip(*run["footprints"]["positions"])
    plt.scatter(pos_x, pos_y, color="g", s=750)

for run in runs:
    if run["collided_at"] is not None:
        plt.scatter(run["collided_at"][0], run["collided_at"][1], color="r", s=750)
    elif run["timed_out_at"] is not None:
        plt.scatter(run["timed_out_at"][0], run["timed_out_at"][1], color="y", s=750)


footprints = runs[0]["footprints"]
obstacles = [[]] * len(footprints["obstacles"][0])
for i in range(len(footprints["obstacles"][0])):
    obstacles[i] = [p[i] for p in footprints["obstacles"]]
# plt.colorbar()
for obs in obstacles:
    # plt.scatter([p[0] for p in obs], [p[1] for p in obs], c=diffs, cmap=cm2, s=250)
    plt.scatter([p[0] for p in obs], [p[1] for p in obs], color="b", s=250)

    # color = "g" if run["success"] else "r"

    # plt.arrow(
    #     start[0],
    #     start[1],
    #     end[0] - start[0],
    #     end[1] - start[1],
    #     head_width=0.1,
    #     head_length=0.1,
    #     fc=color,
    #     ec=color,
    # )
# print(run[index]["actions"])
# vxs, vys, thetas = zip(*run[index]["actions"])
# print(vxs)
# # plt.plot(footprints["time"], vxs, color="r")
# # plt.plot(footprints["time"], vys, color="g")
# plt.plot(footprints["time"], thetas, color="b")
plt.clim(0, 10)
plt.xlim(-9, 9)
plt.ylim(-9, 9)
plt.show()
