#!/usr/bin/env python3
# -*- coding: utf8 -*-
import numpy as np

# Values in 8-bit mode

# threshold - Pixels at or below this level in the input will be 0.
# The range of pixel values between the threshold and saturation will be extended to
# 0 and saturation.
threshold = 32

t = np.zeros((4096,), dtype=np.uint32)

t[: (threshold << 4)] = 0
t[(threshold << 4) :] = np.linspace(0, 4095, len(t[(threshold << 4) :]))

print("LUTValueAll\t0x", "".join(["%08x" % (v,) for v in t]), sep="")
