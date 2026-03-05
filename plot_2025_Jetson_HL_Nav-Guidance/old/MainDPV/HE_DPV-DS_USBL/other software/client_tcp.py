import socket
import sys
import ctypes
import datetime
from utilities import *

if len(sys.argv)<3:
	print "Insert command and Dummy IP"
	sys.exit(0)
	
now = datetime.datetime.now()
ora = now.strftime("%H%M%S")

if sys.argv[1]=='2':
	lung=len(sys.argv[3])+2
	b=chr(int(sys.argv[1]))+ chr(len(sys.argv[3]))+sys.argv[3]
	[d,a]=createRequestMessage(0,1,b,lung)
elif sys.argv[1]=='1':
	b=chr(int(sys.argv[1]))+ chr(6)+ora
	[d,a]=createRequestMessage(0,1,b,8)
else:
	b=chr(int(sys.argv[1]))+'\0'
	[d,a]=createRequestMessage(0,1,b,2)


print ora
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
