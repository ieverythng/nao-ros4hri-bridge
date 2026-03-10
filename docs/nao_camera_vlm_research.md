# NAO Camera VLM/VLA Research

Last updated: 2026-03-09

This note captures the current path for using NAO's built-in cameras for future vision-language and vision-language-action work.

## Bottom Line

Do not build a second camera stack.

`naoqi_driver` already exposes the NAO cameras as standard ROS image topics, and the ROS / ROS4HRI ecosystem already expects that shape.

## Confirmed Existing Interfaces

### NAO camera topics already exist

The `naoqi_driver` ROS 2 repository is maintained and released, and the ROS Index summary describes it as the bridge between ROS 2 and NAOqiOS:

- https://index.ros.org/r/naoqi_driver/

A NAO startup package listing shows these camera topics being published:

- `/nao_robot/camera/front/image_raw`
- `/nao_robot/camera/front/camera_info`
- `/nao_robot/camera/bottom/image_raw`
- `/nao_robot/camera/bottom/camera_info`

Source:

- https://index.ros.org/p/jsk_nao_startup/

This matches what we already observed locally from `naoqi_driver` startup logs in this workspace.

### Existing ROS viewers already fit this path

`rqt_image_view` is already a maintained ROS package and its package description says it displays images using `image_transport`:

- https://index.ros.org/p/rqt_image_view/

`image_view` is also the standard lightweight viewer package:

- https://index.ros.org/p/image_view/

### Standard ROS image transport should be reused

`image_transport` explicitly states that it should be used to subscribe to and publish images, with transparent support for compressed transports:

- https://index.ros.org/p/image_transport/

### Standard ROS -> OpenCV conversion already exists

`cv_bridge` converts ROS 2 image messages to and from OpenCV image representation:

- https://index.ros.org/p/cv_bridge/

## ROS4HRI Relevance

I did not find a generic ROS4HRI VLM package in the sources checked.
The useful pattern instead is:

- camera driver publishes `sensor_msgs/Image` + `sensor_msgs/CameraInfo`
- ROS4HRI perception nodes consume those streams
- higher-level HRI consumers listen to ROS4HRI outputs

Examples:

### `hri_face_detect`

It subscribes to:

- `image` (`sensor_msgs/msg/Image`) or `image/compressed`
- `camera_info` (`sensor_msgs/msg/CameraInfo`)

Source:

- https://index.ros.org/p/hri_face_detect/

### `hri_rviz`

Its Humans overlay is designed for information shown on top of a camera stream, and its example usage expects raw RGB images on `<rgb_camera_stream_ns>/image_raw`.

Source:

- https://index.ros.org/p/hri_rviz/

### `libhri`

For consuming ROS4HRI entities programmatically, the ROS 2 `libhri` repo documents `HRIListener` as the main entry point.

Source:

- https://index.ros.org/r/libhri/

## Practical Path For This Repo

### Phase 1: confirm and inspect camera streams

Use the existing driver topics directly:

```bash
ros2 topic list | rg 'camera/.*/(image_raw|camera_info)'
ros2 topic hz /camera/front/image_raw
ros2 topic echo --once /camera/front/camera_info
ros2 run rqt_image_view rqt_image_view
```

If your driver uses a namespaced prefix such as `/nao_robot/camera/front/image_raw`, keep that namespace and remap consumers to it.

### Phase 2: reuse ROS4HRI perception nodes

The clean next step is not VLM first. It is to prove the camera path with existing ROS4HRI consumers.

Likely first candidates:

1. `hri_face_detect`
2. later `hri_fullbody` if needed
3. `hri_rviz` for visualization on top of the image stream

Example remap shape for face detection:

```bash
ros2 launch hri_face_detect hri_face_detect.launch.py \
  --ros-args \
  -r image:=/camera/front/image_raw \
  -r camera_info:=/camera/front/camera_info
```

This remap is an inference from the published ROS API and should be validated against the exact installed launch file.

### Phase 3: add a thin VLM bridge node

For open-ended vision-language tasks, add one new node instead of a new camera subsystem.

Recommended contract:

- subscribe: `sensor_msgs/msg/Image`
- optional subscribe: `sensor_msgs/msg/CameraInfo`
- convert with `cv_bridge`
- throttle or trigger requests instead of sending every frame
- call the VLM backend
- publish structured outputs, for example:
  - `/chatbot/vision_text`
  - `/chatbot/vision_entities`
  - or ROS4HRI-aligned detections if the output is person/face/body centric

### Phase 4: move from VLM to VLA carefully

For VLA-style control, do not let raw vision output directly actuate the robot.

Use this pattern instead:

- vision node produces grounded observations
- mission/orchestration node decides intent
- existing skill interfaces execute action
  - `/skill/do_head_motion`
  - `/skill/do_posture`
  - `/skill/say`

That keeps the action boundary stable and debuggable.

## Recommended Next Experiment

1. verify the exact front/bottom image topic names on your running robot
2. display them with `rqt_image_view`
3. run one ROS4HRI perception node against the front camera
4. only after that, add a dedicated VLM bridge node in this repo

## Assessment

The camera side is not the blocker.
The likely future work is:

- topic remapping and namespacing discipline
- frame throttling / trigger policy for model calls
- deciding whether vision outputs should become ROS4HRI entities, chatbot context, or both
