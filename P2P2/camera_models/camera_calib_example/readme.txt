Camera calibration example
==========================

This directory documents the expected layout for intrinsic calibration inputs.

The original repository included sample calibration images under
`calibrationdata/`. Those images are intentionally not tracked in this cleaned
open-source tree to keep the repository small.

To run the calibration example, create the directory below and place your own
checkerboard images in it:

    camera_models/camera_calib_example/calibrationdata/

Keep large image datasets outside git, or publish them separately as release
assets if they are needed for reproducible experiments.
