# obj_planner
From package '[obj_planner]()'
# File
`./src/ObjPlannerNode_node.cpp`

## Summary 
 This node impliments a path planner for a vehicle that can detect objects on each side of the road. It rapidly produces
paths formed by pairing these detections across the path.

## Topics

### Publishes
- `/path`: Created path, in the configured path frame.

### Subscribes
- `/tracks`: Poses for each detected object.

## Params
- `debug`: Displays an OpenCV debug window for the convex hull
- `test_latency`: Logs latency taken per planning cycle.
- `path_frame`: Frame the path is transformed into before publishing. Defaults to odom.

