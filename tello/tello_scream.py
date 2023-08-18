import socket
import time
import cv2

# this is UDP client for sending and receiving commands to Tello 

UDP_IP = "172.17.0.2"         # Drone IP
#UDP_IP = "192.168.10.1"
UDP_PORT = 8889                 # command UDP port
 

sock = socket.socket(socket.AF_INET,    # Internet
                     socket.SOCK_DGRAM, # UDP
                     0,                 # not sure if this is needed
                     )

cmd = "command"                     
while True:
    sock.sendto(cmd.encode('utf-8'), (UDP_IP, UDP_PORT))
    data, address = sock.recvfrom(4096)
    reply = data.decode('utf-8')
    if reply == 'ok':
        print('command has been received and acknolodged')
        break

cmd = "streamon"                     
while True:
    sock.sendto(cmd.encode('utf-8'), (UDP_IP, UDP_PORT))
    data, address = sock.recvfrom(4096)
    reply = data.decode('utf-8')
    if reply == 'ok':
        print('streamon has been received and acknolodged')
        break        


# start to capture video
capture = cv2.VideoCapture ('udp:/0.0.0.0:11111',cv2.CAP_FFMPEG)
if not capture.isOpened():
    capture.open('udp://192.168.10.2:11111')

while True:
    ret, frame =capture.read()
    #print(ret)
    if(ret):
        cv2.imshow('frame', frame)
    if cv2.waitKey (1)&0xFF == ord ('q'):
        break
capture.release ()
cv2.destroyAllWindows ()

sock.close()
