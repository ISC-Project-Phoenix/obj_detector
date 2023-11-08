# obj_detector
From package '[obj_detector](https://github.com/ISC-Project-Phoenix/obj_detector)'
# File
`./src/obj_detector.cpp`

## Summary 
 Cone object detection node. It takes depth images of orange cones and outputs points in 3d
where the center of the cone is.

## Topics

### Publishes
- `/object_poses`: 3d Pose of the detected cones. In the reference frame of the camera

### Subscribes
- `/camera/mid/rgb`: RGB images from the camera. Will be rectified.
- `/camera/mid/depth`: Depth images from camera. Assumed rectified.
- `/camera/mid/rgb/camera_info`: Rgb cameras camera info

## Params
- `transport_type`: image_pipeline topic type for rgb. Defaults to raw
- `debug`: Enables debug visualisations
- `test_latency`: Prints e2e latency statistics if true
- `camera_frame`: Frame the poses will be published from
- `area_threshold`: Sets minimum pixel value a contour will return a center
- `aspect_ratio_threshold_min`: Minimum ratio (width/height) of contour 
- `aspect_ratio_threshold_max`: Maximum ratio (width/height) of contour
- `area_perimeter_ratio`: Area/perimeter of the contour 