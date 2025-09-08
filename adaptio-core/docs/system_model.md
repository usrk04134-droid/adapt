# The model

## Coordinate systems

The kinematic model used in this system is a combination of a number of coordinate systems.

### The camera/sensor

The camera coordinate system is defined by the pixels in the image, originating from the top left of the image.

### The image plane

The image plane is a plane located at a distance c from the projection center as described
in [A Comprehensive and Versatile Camera Model for Cameras with
Tilt Lenses](https://link.springer.com/content/pdf/10.1007/s11263-016-0964-8.pdf), figure 15. Looking from the "back" of the
scanner (same direction as the camera is facing) this coordinate system is right-handed with the x-axis oriented to the
right. The origin is defined by the principal point.

### The laser plane (workspace)

This is a plane that is parallel to the actual laser, imagine a plane that is defined by the intersection of the laser
line. Looking from the "back" of the scanner (same direction as the camera is facing) this coordinate system is
right-handed with the x-axis oriented to the right.

### The machine coordinates

This coordinate system is equal to that of the machine.

### The TCP of the first torch

This coordinate system has its origin in the end of the first torch.
