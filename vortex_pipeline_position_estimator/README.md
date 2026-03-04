# vortex_pipeline_position_estimator

Converts 2D pipeline endpoint detections (pixel coordinates) into 3D positions in the world frame. Uses DVL altitude, camera intrinsics, and a flat ground plane assumption to backproject image points to real-world coordinates.

## Overview

This package takes pixel coordinates from the image-space endpoint detection and computes where those points are in the physical world. The output is a 3D landmark in the `odom` frame, suitable for use in navigation and task execution.

```
2D pixel coordinates + DVL altitude + camera pose → [vortex_pipeline_position_estimator] → 3D landmark (odom frame)
```

If both endpoints are provided, the one closest to the world origin is selected and published.

## How it works

1. Receive 2D pixel coordinates from `/pipeline/endpoints`
2. Apply lens distortion correction using camera intrinsics
3. Back-project to a 3D ray in camera frame using the pinhole model
4. Intersect the ray with the ground plane at the DVL-measured altitude
5. Transform the result to the world frame (`odom`) via tf2

The ground plane is assumed to be flat and horizontal at the altitude reported by the DVL.

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Sub | `/pipeline/endpoints` | `vortex_msgs/Point2DArray` | 2D pixel coordinates from image endpoint detection |
| Sub | `/pipeline/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics and distortion model |
| Sub | `/dvl/altitude` | `std_msgs/Float64` | Height above ground in metres (positive) |
| Sub | `/pipeline/camera/segmentation_mask` | `sensor_msgs/Image` | Input image for debug overlay (if debug enabled) |
| Pub | `/orca/landmarks` | `vortex_msgs/LandmarkArray` | 3D pipeline position in `odom` frame |
| Pub | `/pipeline/debug/localization_overlay` | `sensor_msgs/Image` | Debug overlay with projected points (if debug enabled) |

## tf2 Requirements

Requires a valid transform from the `odom` frame to the camera frame (as reported by `CameraInfo.header.frame_id`). The transform is looked up at message timestamp with a 100 ms tolerance.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `endpoints_topic` | string | `/pipeline/endpoints` | Input 2D endpoints topic |
| `dvl_altitude_topic` | string | `/dvl/altitude` | DVL altitude topic |
| `camera_info_topic` | string | `/pipeline/camera/camera_info` | Camera info topic |
| `publish_topic` | string | `/orca/landmarks` | Output landmark topic |
| `enable_debug_image` | bool | `true` | Publish debug overlay image |
| `debug_input_image_topic` | string | `/pipeline/camera/segmentation_mask` | Image to draw debug overlay on |
| `debug_output_image_topic` | string | `/pipeline/debug/localization_overlay` | Debug overlay output topic |

## Launch

```bash
ros2 launch vortex_pipeline_position_estimator position_estimator.launch.py
```
