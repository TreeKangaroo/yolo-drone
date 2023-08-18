import socket
# this is UDP client for sending and receiving commands to Tello 

UDP_IP = "172.17.0.2"         # Drone IP
UDP_PORT = 8889                 # command UDP port
 

sock = socket.socket(socket.AF_INET,    # Internet
                     socket.SOCK_DGRAM # UDP
                     )

cmd = "command"                     
while True:
    sock.sendto(cmd.encode('utf-8'), (UDP_IP, UDP_PORT))
    print('Sending')
    data, address = sock.recvfrom(4096)
    print(data.decode('utf-8'), "\n")
    print('received')
    
sock.close()
