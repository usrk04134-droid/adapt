import re
import subprocess

import matplotlib.pyplot as plt
import numpy as np

# build_macos_arm/data-set-runner --dataset tests/data_set/data_set.yaml --diff 1.18 \
#   --test-app build_macos_arm/test-images

cmd = [
    "build_macos_arm/data-set-runner",
    "--dataset",
    "tests/data_set/data_set.yaml",
    "--diff",
    "1.18",
    "--test-app",
    "build_macos_arm/test-images",
]
output = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
diffs = [line for line in output.split("\n") if "Diff" in line]

m = re.compile(r"\{([^}]+)\}")

all_diffs = np.array([[float(x) for x in m.findall(r)] for r in diffs])

plt.title("Histogram of for all ABW-points in the data set")
plt.xlabel("Distance between calculated and manually estimated point [mm]")
plt.ylabel("Number of points")
plt.hist(all_diffs.flatten(), bins=20)
plt.show()
