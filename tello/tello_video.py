
import time
import cv2



# start to capture video
capture = cv2.VideoCapture ('udp://172.17.0.1:11111',cv2.CAP_FFMPEG)
if not capture.isOpened():
    capture.open('udp://172.17.0.1:11111')

while True:
    ret, frame =capture.read()
    print(ret)
    if(ret):
        cv2.imshow('frame', frame)
    if cv2.waitKey (1)&0xFF == ord ('q'):
        break
capture.release ()
cv2.destroyAllWindows ()
