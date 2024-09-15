import pickle
import sys

import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

run_file = str(sys.argv[1])
index = int(sys.argv[2])

with open(run_file, "rb") as f:
    run = pickle.load(f)

footprints = run[index]["footprints"]
# Plot the footprint

vxs, vys, thetas = zip(*run[index]["actions"])
start_time = footprints["time"][0]
times = [t - start_time for t in footprints["time"]]
plt.plot(times, vxs, color="r")
plt.plot(times, vys, color="g")
plt.plot(times, thetas, color="b")
# Add a legend
plt.legend(["Vx", "Vy", "Theta"])
plt.show()
