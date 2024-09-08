import pickle

import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

with open("run.pkl", "rb") as f:
    run = pickle.load(f)

index = 1

footprints = run[index]["footprints"]
# Plot the footprint

print(run[index]["actions"])
vxs, vys, thetas = zip(*run[index]["actions"])
print(vxs)
# plt.plot(footprints["time"], vxs, color="r")
# plt.plot(footprints["time"], vys, color="g")
plt.plot(footprints["time"], thetas, color="b")
plt.show()
