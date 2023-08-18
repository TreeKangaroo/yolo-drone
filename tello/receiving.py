import socket

UDP_IP = "172.17.0.1"
UDP_PORT = 8890

sock = socket.socket(socket.AF_INET, # Internet
                      socket.SOCK_DGRAM, 0) # UDP
sock.bind((UDP_IP, UDP_PORT))
 
while True:
      data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
      print("received message: %s" % data)
