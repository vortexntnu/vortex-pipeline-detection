# Piplinelinelinefittingpipeline (official name, don't let anyone tell you otherwise)
A package to find two centerlines along a pipeline mask.
## Usage
Lines are represented by endpoints and exported as a posearray and published.
In order to generate a test, you can use a rosbag. When installed, you can launch the bag using something like this
```
ros2 bag play bags/tacc2/tacc2_0.db3 --start-offset 180
```
Make sure the topics match and run
```
ros2 launch pipeline_line_fitting pipeline_line_fitting.launch.py
```
## Tuning

We have the following parameters to be tuned:
| Parameter | Type | Description |
| --- | --- | --- |
| n | int | How many random points to choose to start each iteration |
| k | int | How many iterations of randsac to run |
| t | float | Threshold for points to be considered inliers, for the purpose of scoring |
| fracOfPoints | float | minimum fraction of points that need to bee inliers for a line to be considered |
| removeT | float | Threshold for points to be removed between iterations |
| finalScorethresh | float | required score to be counted as a detected line |
| minTurnAngle | float | minimum angle difference between lines, radians (ofc) |
| size | int | what size of square the image will be resized to during preprocessing |
