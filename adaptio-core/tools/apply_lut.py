#!/usr/bin/python3
# -*- coding: utf8 -*-

import glob
import os

import numpy as np
from PIL import Image

THRESHOLD_VALUE = 32
SCALING_FACTOR = 7
SHIFT_BITS = 3
START_INDEX = 42

files = glob.glob("/Users/erik/Downloads/demo_full/*.tiff")
files.sort(key=lambda f: (os.path.getmtime(f), f))

for f in files[START_INDEX:]:
    print(f)

    im = Image.open(f)
    m = np.array(im).astype(np.uint32)
    m[np.where(m < THRESHOLD_VALUE)] = THRESHOLD_VALUE
    m = (m - THRESHOLD_VALUE) * SCALING_FACTOR
    m >>= SHIFT_BITS
    x = Image.fromarray(m.astype(np.uint8))
    x.save(f)
