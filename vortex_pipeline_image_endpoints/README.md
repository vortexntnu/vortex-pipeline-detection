# vortex_pipeline_image_endpoints

Detects pipeline endpoints in 2D image space from a binary segmentation mask. Outputs pixel coordinates for use by downstream 3D estimation.

## Overview

This package takes a binary mask where the pipeline has been segmented, and finds the relevant endpoint(s) as pixel coordinates. It does not know anything about the physical world — all output is in image space.
Normally used in conjunction with the **vortex_pipeline_position_estimator** package to find a 3D position of the pipeline.

```
segmentation mask (image) → [vortex_pipeline_image_endpoints] → 2D pixel coordinates
```

## Detection Methods

Two methods are supported, selectable via config:

**`lowest_pixel`** (default)
Finds the centroid of the lowest foreground row in the image (highest y-index). Useful for detecting the near end of the pipeline — the point closest to the vehicle.

**`furthest_points`**
Finds the two points with maximum distance along the convex hull of the largest connected component. Returns both ends of the visible pipeline segment.

Both methods apply morphological preprocessing (close + open with a 5×5 elliptical kernel) and isolate the largest connected component before detection.

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Sub | `/pipeline/camera/segmentation_mask` | `sensor_msgs/Image` (mono8) | Binary pipeline mask |
| Pub | `/pipeline/endpoints` | `vortex_msgs/Point2DArray` | Detected endpoint(s) in pixel coordinates |
| Pub | `/pipeline/detector/debug_image` | `sensor_msgs/Image` (BGR8) | Visualization overlay (if debug enabled) |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `detection_method` | string | `"lowest_pixel"` | Detection method: `"lowest_pixel"` or `"furthest_points"` |
| `debug` | bool | `true` | Publish debug visualization image |
| `input_topic` | string | `/pipeline/camera/segmentation_mask` | Input mask topic |
| `output_topic` | string | `/pipeline/endpoints` | Output endpoints topic |
| `debug_topic` | string | `/pipeline/detector/debug_image` | Debug image topic |

## Launch

```bash
ros2 launch vortex_pipeline_image_endpoints image_endpoints.launch.py
```
