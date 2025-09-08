# Scanner

## Introduction

When the scanner is started it continously sends slices (set of 7 ABW points) to the main thread.
They are sent with a 50 ms pace and are sent even if the scanner can not calculate the ABW points for an
image. The confidence in each slice indicates the quality of the ABW points.

If the scanner sent slices with confidence LOW/NO for a period of three seconds it
internally change mode to only use the approximation of ABW0_x/ABW6_x to
calculate the ABW points if the the approximation is available.

### Confidence

Each slice sent from scanner to main have a confidence.

* HIGH
   * Both walls found and the depth on left and right side is at least 7 mm
* MEDIUM
   * Both walls found but depth is less than 7 mm
   * One wall found and the depth on one side is at least 6 mm
   * If approximation of ABW0_x/ABW6_x is used
* LOW
   * One wall found and the depth on both sides are less than 6 mm
   * Zero walls found
* NO
   * Not able to calculate the ABW points for current image and the latest calculated are sent
   * At startup of scanner until a median slice is available. The ABW points is set to 0 in those slices.
