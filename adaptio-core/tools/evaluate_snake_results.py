import matplotlib.pyplot as plt
import numpy as np

with open("results.txt", "r", encoding="utf-8") as f:
    s = f.readlines()


data = [line.split() for line in s[::3]]
path = [d[0] for d in data]
abw = None
for i in range(1, 15):
    row = [float(d[i]) for d in data]
    if abw is None:
        abw = row
    else:
        abw = np.append(abw, row)

abw = abw.reshape((14, -1))

# abw0 = np.array()
# abw0_y = np.array([float(d[2]) for d in data])
# abw1_x = np.array([float(d[3]) for d in data])
# abw1_y = np.array([float(d[4]) for d in data])
# abw2_x = np.array([float(d[5]) for d in data])
# abw2_y = np.array([float(d[6]) for d in data])
# abw3_x = np.array([float(d[7]) for d in data])
# abw3_y = np.array([float(d[8]) for d in data])
# abw4_x = np.array([float(d[9]) for d in data])
# abw4_y = np.array([float(d[10]) for d in data])
# abw5_x = np.array([float(d[11]) for d in data])
# abw5_y = np.array([float(d[12]) for d in data])
# abw6_x = np.array([float(d[13]) for d in data])
# abw6_y = np.array([float(d[14]) for d in data])
num_walls = np.array([int(d[15]) for d in data])
area = np.array([float(d[16]) for d in data])

width = abw[12] - abw[0]
x = np.arange(len(width))
left_height = abw[1] - abw[3]
right_height = abw[13] - abw[11]
plt.subplot(3, 1, 1)
plt.plot(x, width, "b.")
plt.subplot(3, 1, 2)
plt.plot(x, left_height, "r.", x, right_height, "g.")

# plt.plot(x[np.where(num_walls == 2)], width[np.where(num_walls == 2)] , 'r.')
# plt.plot(x[np.where(num_walls == 1)], width[np.where(num_walls == 1)] , 'g.')
# plt.plot(x[np.where(num_walls == 0)], width[np.where(num_walls == 0)] , 'b.')
# last_with_two_walls = np.where(num_walls == 2)[-1][-1]
# low_width = np.where(left_height < 0.005)[-1][-1]


# print(path[low_width], width[low_width], num_walls[low_width])
# plt.show()


plt.subplot(3, 1, 3)
m = np.vstack((abw[0, 6:], abw[0, 5:-1], abw[0, 4:-2], abw[0, 3:-3], abw[0, 2:-4], abw[0, 1:-5], abw[0, :-6]))
# m = np.vstack((abw0_y[6:], abw0_y[5:-1], abw0_y[4:-2], abw0_y[3:-3], abw0_y[2:-4], abw0_y[1:-5], abw0_y[:-6]))
# m = np.vstack((abw1_x[6:], abw1_x[5:-1], abw1_x[4:-2], abw1_x[3:-3], abw1_x[2:-4], abw1_x[1:-5], abw1_x[:-6]))
print(m.shape)
q = np.median(m, axis=0)
print(q.shape)
for i in range(7):
    mx = np.vstack(
        (
            abw[i * 2, 6:],
            abw[i * 2, 5:-1],
            abw[i * 2, 4:-2],
            abw[i * 2, 3:-3],
            abw[i * 2, 2:-4],
            abw[i * 2, 1:-5],
            abw[i * 2, :-6],
        )
    )
    my = np.vstack(
        (
            abw[i * 2 + 1, 6:],
            abw[i * 2 + 1, 5:-1],
            abw[i * 2 + 1, 4:-2],
            abw[i * 2 + 1, 3:-3],
            abw[i * 2 + 1, 2:-4],
            abw[i * 2 + 1, 1:-5],
            abw[i * 2 + 1, :-6],
        )
    )
    qx = np.median(mx, axis=0)
    qy = np.median(my, axis=0)
    q = np.sqrt(np.diff(qx) * np.diff(qx) + np.diff(qy) * np.diff(qy))
    m = np.sqrt(np.diff(abw[i * 2]) * np.diff(abw[i * 2]) + np.diff(abw[i * 2 + 1]) * np.diff(abw[i * 2 + 1]))

    print(i, np.max(q), np.max(q), np.mean(m), np.mean(m))

# plt.plot(x[1:], np.diff(abw0_x), 'b.')
plt.plot(x[1:], np.diff(abw[0]), "b.")
# plt.plot(x[1:], np.diff(abw1_x), 'b.')
plt.plot(x[3:-5], np.diff(q), "r.")

plt.show()
