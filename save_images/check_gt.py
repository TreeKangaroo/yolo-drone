# check the ground truth of the video frames

from tello_gt import *
import cv2
import random

vc_prefix = 'vc05f'
first = 0
last = 7619

num = 10

for k in range(num):
    index = random.randint(first, last)
    if vc05f_gt(index):
        print('Tello in the frame', index)
    else:
        print('No Tello', index)
        
    img_file = './trial5/image/'+vc_prefix+str(index)+'.jpg'
    img = cv2.imread(img_file)
    cv2.imshow('frame', img)
    cv2.waitKey(0)

cv2.destroyAllWindows()
