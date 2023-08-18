import os
import cv2
import pyrealsense2 as rs
import numpy as np
import time
                   
                    
def draw_progress_bar(image, n, m):
    # calculate progress
    p=480-int(480*n/m)
    
    # draw progress bar
    start_point=(620, 480)
    end_point=(635, p)
    # Blue color in BGR
    color = (255, 0, 0)
    # Line thickness of -1 to fill the bar
    thickness = -1
    image = cv2.rectangle(image, start_point, end_point, color, thickness)
    
    # draw bar for remaining tasks
    start_point=(620, p)
    end_point=(635, 0)
    # Blue color in BGR
    color = (255, 255, 255)
    # Line thickness of -1 to fill the bar
    thickness = -1
    image = cv2.rectangle(image, start_point, end_point, color, thickness)
    
def main():
    
    path = '/home/michelle/Pictures/test_data/trial1'
    cnt = 0
    cnt2 = 0
    
    # check if folder exists
    index = 0
    if os.path.exists(path):
        for f in os.listdir(path + '/image'):
            if f[-4:] == '.jpg':
                num = int(f[5:-4])
                if index <= num:
                    index = num + 1
    else:
        os.makedirs(path)
        os.makedirs(path + '/image')
        os.makedirs(path + '/depth')
        os.makedirs(path + '/bbox')

    # start realsense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
    profile = pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            
            # discard the first 20 images
            if cnt2 < 20:
                cnt2 += 1
                continue
            
            # convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # save image and depth data
            image_name = path + '/image/' + 'image' + str(index) + '.jpg'
            cv2.imwrite(image_name, color_image)
            data_name = path + '/depth/' + 'image' + str(index) + '.npy'
            np.save(data_name, depth_image)
            
            # update index and cnt
            index += 1
            cnt += 1
            time.sleep(0.01)
            
            # draw images with progress bar
            #draw_progress_bar(color_image, cnt, num_images)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))
            
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        
if __name__ == "__main__":
    main()    
