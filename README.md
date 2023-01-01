# RANSAC-Plane-Fitting
Custom function to implement Random Sample Consensus (RANSAC) to fit a plane in 3d point cloud. 

# RANSAC

Using demo point cloud available in open3d, this project implement a custom function to fit a plane in the 3d point cloud.

## Process
* Select random three points to form a planar surface.
* Find equation of plane passing through selected points.
* Count the number of inliers satisfying equation of the plane.
* Repeat the previous steps for specific number of iterations.

## Results
