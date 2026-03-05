import socket
import sys
import ctypes

if len(sys.argv)<3:
	print "Insert command and Dummy IP"
	sys.exit(0)

libmean = ctypes.CDLL('./libutilities_u.so')
a="\0" * 10
b=chr(int(sys.argv[1]))+'\0'
createRequestMessage=libmean.createRequestMessage
createRequestMessage.argtypes=[ctypes.c_char_p,ctypes.c_byte,ctypes.c_byte,ctypes.c_char_p,ctypes.c_byte]
print libmean.createRequestMessage(a,0,1,b,2)
print ":".join("{:02x}".format(ord(c)) for c in a)

hostname, sld, tld, port = 'www', 'integralist', 'co.uk', 80
target = '{}.{}.{}'.format(hostname, sld, tld)

# create an ipv4 (AF_INET) socket object using the tcp protocol (SOCK_STREAM)
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# connect the client
# client.connect((target, port))
client.connect((sys.argv[2],  11999))

# send some data (in this case a HTTP GET request)
client.send(a)

# receive the response data (4096 is recommended buffer size)
response = client.recv(4096)

print ":".join("{:02x}".format(ord(c)) for c in response)
