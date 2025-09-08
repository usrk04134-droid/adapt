# Image processing

This document describes the image analysis changes from the "Teknisk beskrivning Adaptio".

## Centroids for surface lines

For each column:

- starting at the bottom row, find the first instance of the column's maximum value.
- get the centroid (center of mass) of a small region around this maximum.
- if the maximum value is below the gray_minimum_top value from the scanner configuration, the centroid value is considered "not found"
- otherwise, the centroid value is returned.

## Convert to LPCS

Centroid values are converted to LPCS. All calculations are then done in double precision LPCS coordinate values, in metres.

## Surface lines, first pass

In `SurfaceLines`: start by getting an educated guess of the left surface end point (ABW_0_x). The value from the previous pass is used if we have it, otherwise we use

```text
left surface end point = the left start of the FOV + (the total width of the FOV - the width of the joint)/2
```

We then run a fast RANSAC FitPoints to get slope and offset of the surface line. The right surface is found analogously.

## Surface lines, second pass

In `GetImprovedSurfaceLine`: starting at the left side, compare the centroid value in each column with the ideal line found in the first step. If we are within tolerance (less than 2 mm above and less than 4 mm below) we add that column to the list of "ok" indices. If we have found at least 20 indices and then find 5 indices in a row that are *below* the ideal line, then we break.

The list of "ok" indices are now again used for a RANSAC FitPoints with lower tolerance to get a new improved line. Finally, the maximum x of the line inliers are used to get a better value for the left surface end point.

## Groove walls - preparation

Using the corner end point from the previous point, and the joint angle (the median angle from the previous pass if available, otherwise the angle from the joint geometry) we estimate a left wall line in `GetWallCoordsLaserPlane`.

We then translate the left surface line down "left groove depth" (the median depth from the previous pass if available, otherwise the depth from the joint geometry). The intersection of the left wall line and this translated line is the lower right corner of the wall.

Finally, the corners are adjusted (in `TransformCoordsImagePlane`) - the top corner 2 mm down, the bottom corner 3 mm up. We translate back to image coordinates and cut out this portion of the source image (in `GetWallImages`). The cutout is extended 30 pixels into the joint (to account for bevelling, where the top surface at the transversal joint has been cut a bit to ensure a good weld result when the cap is welded).

(The Python version also has a quite complicated "mask" applied to the wall image, this step was quite fragile and also does not seem to affect the result very much. The masking step has been removed.)

## Groove walls - centroids

The wall images from the previous step are used to determine the wall centroids (in `GetWallCentroids`). The centroid search method is the same as for the top lines, but we go by row instead of column. The search is "from the outside" - from the left side for the left wall image and from the right side for the right wall image. Also, the threshold value for the centroid max value is gray_minimum_wall..

## Groove lines

In `GrooveLines` we use the centroids found in each wall image to do a RANSAC FitPoints to get the wall lines.

## Fit groove line

We now calculate a piece-wise linear wedge (PWLWedge) for the joint, in `FitGrooveLine`. In Python, this is done using a ceres solver for the the top corner points, the opening angle of the wedge and the rotation of the wedge. With sharpened tolerances in finding the groove walls, this is no longer neccessary and actually detrimental to the result.

Instead, the top corners of the wedge are found using the intersection of the left wall and the left top line, and right wall and right top line, respectively. The bottom vertex of the wedge is found using the intersection of the left and right walls.

In this step we also calculate the wall angles and push them to the angles buffer.

## Get bottom centroids

In `GetBottomCentroids` we take

- the maximum x value of the left wall inliers
- the minimum x value of the right wall inliers
- the highest of the minimum y values of the wall inliers
- the y value of the left corner point minus the preconfigured joint depth minus 2 mm

These four limits are transformed from LPCS to image coordinates. A subimage is created using these four coordinates and the centroids are calculated in the same way as for the surface lines (but with a potentially different centroid threshold).

## GrooveBottomLine

The centroids are filtered to only include those within the triange of the wedge. Next, the y coordinates are median filtered with a quite large window size. If enough coordinates remain,

If not enough coordinates remain at any point, finding the bottom line fails.

## Extract ABW points

If no bottom line was found, the ABW points are calculated using the wedge corners. This is typically the case for empty v-joints.

The other code (i.e. when a bottom line was found) is currently not very well understood
