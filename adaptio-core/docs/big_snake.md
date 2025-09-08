# Big snake

## Introduction

The "big snake" algorithm, initially implemented in November 2024, is a completely new algorithm for extracting the ABW points from an image. The basic idea is to follow the laser line/trace across the image from one side to the other, and detecting where the angle of the trace matches the pre-defined wall angle.

## Sequence

### Create a mask for the joint

If there is a previous set of ABW points, we construct 4 triangles from these points:

- ABW0-ABW1-ABW6 (points offset right, up, and left, respectively)
- ABW0-ABW5-ABW6 (points offset right, up, and left, respectively)
- ABW0-(ABW0_x,ABW1_y)-ABW1 (points offset down, none, and left, respectively)
- ABW6-ABW5-(ABW6_x,ABW5_y) (points offset down, right, and none, respectively)

We then "paint" a mask with 1's outside the triangles and 0's inside. The goal here is to reduce impact of the reflections inside the joint and below/outside the joint walls.

### Trace the laser line from left to right

Get the starting pixel by

- Identify the first column with a pixel above a configurable threshold value.
- Get the row with the maximum value in that column. If multiple rows share the same value, choose the median y value.

The starting angle is set to 0.

From each pixel, find the angle to the next pixel on the line:

- Step a configurable distance along the current angle from the current pixel
- Search the surrounding area for pixels above the threshold where the difference between the current angle and the angle to the current pixel is low enough (see "surface mode" below).
- Exclude pixels with a '0' mask value, if a mask could be constructed (meaning that a previous profile exists)
- Calculate the centroid angle for all such pixels
- The actual next angle is a mix between the above angle and the previous angle. The weight distribution between them is determined by "surface mode", see below.

The next pixel is calculated based on the current pixel, the next angle and a preset move distance. If there are no pixels above the threshold in the search area then we have "lost track". We continue along the same angle for a bit, and then abort if we don't recover. This typically happens due to having followed a strong reflection line at the groove bottom.

Surface mode: we begin each search in surface mode. Surface mode does three things

1. Decrease how much to the left and right we look
1. Decrease how high an area around current_point we should look in
1. React more slowly to changes in angle

We leave surface mode (and don't enter it again) if either:

1. No pixels in the search area are above the threshold (even once). This typically happens if the wall is faint so that we miss it completely and continue "into the air".
1. The absolute value of the angle (compared to straight ahead) is more than 0.3 * 𝛑/2. When this happens we have found and begun the wall.

Note that in general we will overshoot the corner a bit due to the surface mode. To address this we discard the latest points when leaving surface mode (effectively rewinding the search a bit).

### Trace the laser line from right to left

This is done analogously to the previous step but looking for a starting pixel from the right side and with an initial angle of 𝛑.

### Merge the two traces

First

- check that the two traces/snake meet in the middle. If not (for example due to flux in the joint) we fail.
- check if the left snake terminates early, and if so, clip left to where they first meet from the perspective of the right snake
- as above but for the right snake

Then we handle the overlapping parts (it is common for the left and right snake to overlap completely if they both run from side to side without terminating early):

- average them together, giving the left snake more weight at the left side and the right snake more weight at the right side
- smoothen the overlap using a least squares fit around each overlap point. Without this the angle between each point will be very noisy and hard to use for wall detection.

The final snake is the union of the left part without overlap, the processed overlap, and the right part without overlap. Since the right snake starts from the right it needs to be reversed in order to get the snake in ascending x order.

## Convert to LPCS and calculate angles

The finished snake is converted to LPCS using the standard function. The angle between each consecutive point in LPCS is calculated.

### Detect left wall, left top surface and ABW0,1

To detect the left wall we go through the list of angles and find all indices where the angle is close to the left wall angle (meaning the previous wall angle, if defined, clipped to the joint geometry wall angle +/- half the groove angle tolerance). Then we calculate the median index which is assumed to be a point on the wall (otherwise there are other similar slopes in the snake larger than the wall).

For example, assume that the index list matching the left wall angle is [34,78, 215, 218, 219, 220, 221, 224, 225, 226, 229, 800, 801]. The median will be index #6, with value 221. Next we include indices above and below #6 so that the step (in value) is no more than 5. This will get us [215,218,219,220,221,224,225,226,229], meaning indices #2 up to and including #10. The wall is assumed to run from 215 to 229.

If a wall is detected (there might not be enough indices matching the left wall angle), we fit a line to the points in the wall, fit a line to the points from the start of the snake to the left edge of the wall, and get the intersection. This is ABW0.

We then translate the wall line a bit horizontally, locate the intersection with the snake, and then translate that intersection back, to find ABW1.

A special check is made, that ABW0 is not allowed to deviate horizontally more than MAX_HORIZONTAL_MOVEMENT from the previous ABW0. If it does, then these ABW0 and ABW1 are discarded.

## Detect right wall, right top surface and ABW6,5

This is done analogously to the previous step.

## Check joint width criterium

If both walls were found, we check that the distance between them is within tolerance (currently 5 mm) from the set joint geometry. Otherwise we discard the walls and continue as if no walls were found.

## Calculate ABW0,1,5,6 if just one wall was found

If just one wall was found, the other two ABW points are calculated. This uses the groove width from the median profile from the last few frames if available, otherwise the groove width from the joint geometry. The opposite top corner Q is calculated from the top corner P and the width W. Then the remained of the snake from this corner out to the edge is used for a FitPoints line estimation for a surface line. The end point of the surface line is used as a basis for a new corner point Q'. Q' is clamped to Q +/- 0.5 * MAX_HORIZONTAL_MOVEMENT, in case the surface "tries" to move to far.

At this point we check that ABW1 is to the right of ABW0 and ABW5 is to the left of ABW6, and if not, return an error. This can happen if the top surface extends out into the joint from ABW0 or ABW6, and especially if it moves up a bit. This typically happens if we incorrectly follow reflections near the corner out into the joint.

Finally, we check that if there is a previous profile, the wall height has not deviated too much from the previous wall height. This check is made on either side.

## Calculate ABW0,1,5,6 if no walls were found

If no walls were found, we assume that the joint is the part of the snake that is least smooth. Mathematically, we compare parts of the snake of width W and get the standard deviation of each part. An iterative search method is used to find the [x_0, x_0 + W] region with the highest standard deviation.

After x_0 is found ABW0 is extracted by getting the end point of the surface up to x_0 (using FitPoints). If the end of the surface is too far away from x_0 we clamp ABW0 to be closer to x_0. ABW6 is calculated analogously but instead of starting at x_0 + W we start at ABW0 + W. This introduces a slight asymmetry in the calculations; ABW0 and ABW6 are not calculated using exactly the same algorithm.

ABW1 and ABW5 are calculated using ABW0, ABW6, and the known joint angles.

## Check surface angle tolerance

As a sanity check we determine the angle from ABW0 to the starting point of the snake (here an average of initial y values are used to reduce noise) and check that the absolute value of the angle is less than 0.5 times the surface angle tolerance. Analogously for the right surface

## Find bottom intersects to get ABW2,3,4

The x coordinates for ABW1 and ABW5 are used to calculate the x coordinates for ABW2,3,4. Finally, the y coordinates of ABW2,3,4 are calculated using the vertical intersection of the snake at these x coordinates.
