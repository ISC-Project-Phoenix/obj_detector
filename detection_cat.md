# detection_cat
From package '[obj_detector](https://github.com/ISC-Project-Phoenix/obj_detector)'
# File
`./src/DetectionCatNode_node.cpp`

## Summary 
 Node that concatinates the detections of two detectors together, emulating one "big camera".

## Topics

### Publishes
- `/object_detection_cat`: Concatinated points, in the common frame and with overlapping detections removed.

### Subscribes
- `/object_detection1`: First detection source
- `/object_detection2`: Second detection source

## Params
- `common_frame`: Common frame to transform both topics into.
- `overlap_tolerence`: Overlapping detections under distince tolerence will be combined into one.

