import multiprocessing
import os

import cv2
import numpy as np


def process_image(line):
    line = line.split()
    path = line[0]
    data = [float(x) for x in line[1:]]
    offset = int(data[16])
    abw_x = np.round(np.array(data[17:24])).astype(np.uint32)
    abw_y = np.round(np.array(data[24:31])).astype(np.uint32)
    centroids = np.round(np.array([float(x) for x in data[31:]]).reshape((2, -1))).astype(np.uint32)

    print(path)
    i = cv2.imread("../images/weld_test_imgs/" + path)

    for x, y in zip(centroids[0, :], centroids[1, :]):
        i = cv2.circle(i, (x, y + offset), radius=3, color=(255, 0, 0), thickness=-1)
    for x, y in zip(abw_x, abw_y):
        i = cv2.circle(i, (x, y + offset), radius=10, color=(0, 255, 255), thickness=3)

    i = cv2.putText(i, path, (10, 70), fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=1, color=(0, 255, 255), thickness=2)

    cv2.imwrite("sequence/" + path, i)


if __name__ == "__main__":
    with open("results.txt", "r", encoding="utf-8") as f:
        s = (line for line in f)

    with multiprocessing.Pool() as pool_obj:
        ans = pool_obj.map(process_image, s)

    os.system("for i in $(ls sequence/*.tiff | sort -n); do touch $i; done")
