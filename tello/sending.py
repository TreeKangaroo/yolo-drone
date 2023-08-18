import socket

UDP_IP = "172.17.0.2"
UDP_PORT = 8889
MESSAGE = b"Hello, World!"
 
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)
 
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

while True:
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    print('Sending')

