"""dart_sim: standalone synthetic dart scene + projection model.

This package is simulation only.  It has no dependency on any measurement
package: it publishes a fake camera stream on ``/camera/image_raw`` (the same
topics a real camera driver uses) plus hidden ground truth on ``/sim/truth/*``.
The measurement code can be developed against it, then pointed at a real
camera without changes.
"""
