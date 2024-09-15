import re
import sys

from velmwheel_gym.types import Point

benchmark_file = str(sys.argv[1])
benchmark_file_dynamic = str(sys.argv[2])

with open(benchmark_file, "r") as f:
    benchmark = f.readlines()

with open(benchmark_file_dynamic, "r") as f:
    benchmark_dynamic = f.readlines()


points = []
results = []
for run in benchmark:
    run = run.strip()
    point, status = run.split(":")
    results.append(1 if "success" in status else 0)
    points.append(point)

points_dynamic = []
results_dynamic = []
for run in benchmark_dynamic:
    run = run.strip()
    point, status = run.split(":")
    results_dynamic.append(1 if "success" in status else 0)
    points_dynamic.append(point)

# Plot points and results using a bar plot
# Should show numbers
# But there should be a legend mapping numbers to points
import matplotlib.pyplot as plt

# Map points to indices
point_to_index = {point: index for index, point in enumerate(points)}
index_to_point = {index: point for point, index in point_to_index.items()}

print(points[-1])
numbers = [point_to_index[point] for point in points]
numbers = sorted(numbers)
print(len(numbers))

# Sum results for each point

# results2 = [
#     sum([results[i] for i, point in enumerate(points) if point == p]) for p in points
# ]
results2 = [
    sum([results[i] for i, point in enumerate(points) if point == index_to_point[idx]])
    for idx in numbers
]

results_dynamic2 = [
    sum(
        [
            results_dynamic[i]
            for i, point in enumerate(points_dynamic)
            if point == index_to_point[idx]
        ]
    )
    / 5.0
    for idx in numbers
]

print(len(results_dynamic2))

import numpy as np

x = np.arange(len(numbers))  # the label locations
width = 0.2  # the width of the bars
multiplier = 0

fig, ax = plt.subplots(layout="constrained", figsize=(10, 5))

data = {
    "Środowisko z przeszkodami ruchomymi": results_dynamic2,
    "Środowisko bez przeszkód ruchomych": results2,
}

for name, res in data.items():
    offset = width * multiplier
    rects = ax.bar(x + offset, res, width, label=name)
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel("Współczynnik sukcesu", fontsize=24)
# ax.set_xticks(x + width, numbers)
ax.legend(loc="upper left", fontsize=24)
# Increase font size in y axis


# The bar plot should sum up the results for each point
# plt.bar(numbers, results2, color="skyblue")
# Legend that maps numbers to points
plt.show()
