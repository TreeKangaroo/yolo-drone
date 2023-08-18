#!/usr/bin/env python3

import pyrealsense2 as rs

pipe = rs.pipeline()
 

cfg = rs.config()

cfg.enable_stream(rs.stream.color,640, 480, rs.format.bgr8, 30)
 

prof =pipe.start(cfg)
 

ds = prof.get_stream(rs.stream.color)
 

intr = ds.as_video_stream_profile().get_intrinsics()
print(intr)
