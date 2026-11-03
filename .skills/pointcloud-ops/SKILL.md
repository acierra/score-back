---
name: pointcloud-ops
description: PointCloud processing operations, builder architecture, and test verification guide.
---

# PointCloud Operations Skill

## Overview
This skill provides guidance for developing and testing PointCloud operations in CloudPointRPC.

## Key Guidelines
1. **Source Code**: `src/cloud_point/point_cloud_builder.cpp` and `include/cloud_point/point_cloud_builder.hpp`.
2. **Reprojection**: Stereo images are converted into 3D points via `cv::reprojectImageTo3D`.
3. **Verification**: Run unit tests using `build/tests/unit_tests --gtest_filter=*PointCloud*`.
