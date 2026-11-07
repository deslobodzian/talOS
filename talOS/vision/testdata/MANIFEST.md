# talOS Vision Testdata Manifest

This directory contains photographic fixtures, canonical tag family samples, and synthetic test scenes used for regression and differential parity testing of the AprilTag detection and pose estimation pipeline in talOS.

`2026-rebuilt-welded.json` is the shared 2026 field layout fixture copied from
the robot vision data. Framework simulation consumes this package's exported
fixture, never a dependency on robot code.

Pose estimation uses OpenCV optical camera coordinates and NWU robot mounting
extrinsics. Layout tag corners lie in the x=0 plane, in detector index order;
distorted observations are undistorted into pinhole pixels before homography
decomposition and joint multi-tag PnP. Reported uncertainty scales quadratically
with range and inversely with the square root of the number of observed tags.

## 1. Photographic Fixtures

The three photographic test images and their expected detection ground truth files are sourced from the upstream AprilRobotics/apriltag repository (`test/data/` at release v3.4.5):

- `33369213973_9d9bb4cc96_c.jpg` / `33369213973_9d9bb4cc96_c.txt`
- `34085369442_304b6bafd9_c.jpg` / `34085369442_304b6bafd9_c.txt`
- `34139872896_defdb2f8d9_c.jpg` / `34139872896_defdb2f8d9_c.txt`

### License and Attribution

The photographic images were taken at the NASA Swarmathon competition at Kennedy Space Center (KSC).
Licensed under **Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)**.
Source URLs:
- https://www.flickr.com/photos/nasakennedy/33369213973/
- https://www.flickr.com/photos/nasakennedy/34085369442/
- https://www.flickr.com/photos/nasakennedy/34139872896/

Upstream reference: [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag) test suite.

## 2. Canonical Tag Family Samples (`tag36h11`)

Sample canonical tag images from the `tag36h11` family sourced from [AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs):

- `tag36_11_00000.png`: Tag ID 0 (10x10 px, 1-bit border, 8x8 payload)
- `tag36_11_00001.png`: Tag ID 1 (10x10 px, 1-bit border, 8x8 payload)
- `tag36_11_00002.png`: Tag ID 2 (10x10 px, 1-bit border, 8x8 payload)
- `tag36_11_00003.png`: Tag ID 3 (10x10 px, 1-bit border, 8x8 payload)
- `tag36_11_00004.png`: Tag ID 4 (10x10 px, 1-bit border, 8x8 payload)

## 3. Synthetic Test Scenes

Deterministic test scenes generated for testing detection, corner extraction, homography, and non-aligned strides:

- `scene_single_tag0.png`: 640x480 scene containing tag 0 centered at (240, 160) with size 160x160 px.
- `scene_multi_tag.png`: 800x600 scene containing tags 0, 1, 2, 3 in four quadrants.
- `scene_rotated_tag.png`: 640x480 scene containing tag 4 rotated by 30 degrees.
- `scene_non_aligned.png`: 643x487 scene with odd dimensions (not divisible by 16 or standard GPU threadgroups) containing tag 0 at (200, 150) with size 150x150 px.
- `scene_empty.png`: 640x480 blank scene testing negative detection (zero false positives).

## 4. SHA-256 Checksums

```text
6f2888334e65440576a6424928a230b82461b1a0a42d87d3b8835d6385901481  33369213973_9d9bb4cc96_c.jpg
0237b369c1873bd1e603d185573266030e59945cbe7f55ddf958337eaaefd002  33369213973_9d9bb4cc96_c.txt
386c70bbd4d98dc0954d053240e73824fa8e07bdf8dc7f99ea50cac872b9b065  34085369442_304b6bafd9_c.jpg
521300049f229c66f98ac35b9354b1a638ed8dd5724d20a96f17e21625905eb6  34085369442_304b6bafd9_c.txt
435a619d642e2d50fb32224dc8e12992b08c2a596b02deee3fba497d3bff672a  34139872896_defdb2f8d9_c.jpg
2361b3340286253eb67e8cfa467a6e9bb02e49b11d1ee8ee117aae9614bbf9bb  34139872896_defdb2f8d9_c.txt
029719a802c2ce1056955330a690b4ae2170a1195aa0ca914c2d6b6308621748  scene_empty.png
cd0fa0409d6328178188fad1cf084f4433e91c83625fc024b1d58e85e95caed1  scene_multi_tag.png
addc9e7da32477b3ac724b88e767d2c553b2daf28172ccc9666c8059428b87ff  scene_non_aligned.png
9b68e0b8ffbd3fd0fc460eb16c5020766574e06da059fa0106535e42afdc68b2  scene_rotated_tag.png
1916dfd06ec050f66132a4fe68885f6bb42f063b932c78cc917659c7318e3f07  scene_single_tag0.png
48d811f770fc3595aaf5650a5fd8007843e7fece1f8fdcb94e0f113b4c9ed0c6  tag36_11_00000.png
2b30c316b3f8f32d80ba4e83feee8471c41a04aeba75bf1a23f1cf9c990394ce  tag36_11_00001.png
56c1561f55d075bc256cc5dddd44ca067d979095d9147f423b6672a4cf144957  tag36_11_00002.png
1faf4f5388680a6f9f54f2c67d47570dd043aad67f206ca2f429d3a21d77ff94  tag36_11_00003.png
1031c40f2c885e8646fae72bb147a12ba328b66d02c2dd59c7ee2a727d217c1c  tag36_11_00004.png
```

