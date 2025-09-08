import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches

a = np.fromfile("snake_test_angles.txt", sep=" ")
m_lpcs = np.fromfile("snake_in_lpcs.txt", sep=" ").reshape((3, -1))

x = m_lpcs[0, :]
y = m_lpcs[1, :]

x_min = np.min(x)
x_max = np.max(x)
y_min = np.min(y)
y_max = np.max(y)

width = 0.053
x0 = x_min + 0.5 * width
xs = []
ls = []
while x0 < x_max - 0.5 * width:
    in_range = np.where(abs(x - x0) < 0.5 * width)
    s = np.std(a[in_range[0]])
    xs.append(x0)
    ls.append(s)
    x0 = x0 + 0.0005

q = np.argmax(ls)
rect = patches.Rectangle(
    (xs[q] - 0.5 * width, y_min), width, y_max - y_min, linewidth=1, edgecolor="r", facecolor="none"
)

plt.subplot(3, 1, 1)
plt.title("Joint (laser plane coordinates)")
plt.plot(x, y, ".")
plt.gca().add_patch(rect)
plt.xlabel("x")
plt.ylabel("y")
plt.subplot(3, 1, 2)
plt.title("Angle (in laser plane)")
plt.plot(x[:-1], a, ".")
plt.xlabel("x")
plt.ylabel("angle")
plt.subplot(3, 1, 3)
plt.xlabel("x, window center")
plt.ylabel("σ(angle)")
plt.plot(xs, ls)
plt.show()
