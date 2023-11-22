# calibration

A simple opencv wrapper for calibration, combined with intellegent methods of forming images from events that enables calibration

### Usage
`calibration`

###

There were guidelines to do stereo calibration in the internet. Few of them listed here.

-  Specific distance of the calibration pattern from the camera shouldn't really matter. Calibration must be performed with fixed focus.
-  Having a checkerboard with more number of squares is beneficial as there are more corner points to extract. This enables us to get more 3D-2D point correspondences (Size of square shouldn't make a difference).
-  Different points of views and angles needed. Calibration detects focal length and distortion parameters by least square method, for which different angles of the checkerboard are needed to obtain a better solution.
-  Enough wide border around the calibration pattern is required.
-  A large pattern is better for cameras with larger baseline, as stereo overlap is lesser.
-  Some calibration toolboxes require the input of checkerboard dimensions to be either odd number x even number (eg 9x6) or vice versa (eg 8x7).
-  Preferably the checkerboard should cover at least half of the area of the image.
-  The calibration pattern must be moved to cover the entire FOV of the camera to obtain better calibration.Skew helps in focal length determination and moving the checkerboard towards the corners helps in determining the distortion co coefficients.
-  Good lighting conditions is of at most importance and is often overlooked.
-  Some sources say circles are easier to localize than corner points and using a circles pattern might lead to better calibration.
-  Humidity changes might affect normal paper that is used as it absorbs moisture. Thick paper must be used and the calibration pattern must be printed using a Laser printer and should be stuck on a glass backing preferably.

