import socket
import sys

if len(sys.argv)<2:
	print "Insert server IP address"
	sys.exit(0)

localIP     = sys.argv[1]
localPort   = 20001
bufferSize  = 1024

msgFromServer       = "#DS_MES"
bytesToSend         = str.encode(msgFromServer)
 
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")

# Listen for incoming datagrams
while(True):
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]
    clientMsg = "Message from Client: {}".format(message)
    clientIP  = "Client IP Address: {}".format(address)
    print(clientMsg)
    print(clientIP)
	# Sending a reply to client
    UDPServerSocket.sendto(bytesToSend, address)
