
BROADCAST =0xF
MAXADDRESS =0x4F
MAXNUMDEVICES= 32

class stateMachine ():
	state=0
	message=''
	pointer=0
	counter=0
	dataLength=0
	totalLength=0
	typ=0
	address=0
	vendor=0
	model=0
	command=''
	
    # _fields_ = [
        # ("state", ctypes.c_int),("message",ctypes.c_char * 263),("pointer", ctypes.c_int),("counter", ctypes.c_int),
        # ("dataLength", ctypes.c_char),("totalLength", ctypes.c_char),("type", ctypes.c_char),("address", ctypes.c_char),
        # ("vendor", ctypes.c_char),("model", ctypes.c_char),("command", ctypes.c_char),
    #]
    
class unit ():
	typ=0
	address=0
	vendor=0
	model=0
	# _fields_ = [
		# ("type", ctypes.c_char),("address", ctypes.c_char),("vendor", ctypes.c_char),("model", ctypes.c_char),
		# ]
	
def executeStateMachine(request, s, mode):
	ret = 0
	state=0
	dataLength=0
	totalLength=0
	command=''
	s.message=''
	for d in request:
		c=ord(d)
		if state==0:
			s.pointer = 0
			if((mode == 0 and c == 0xFA) or (mode == 1 and  c== 0xFD)):	
				#print 'ok'
				state = 1
				s.counter = 0
				s.message+=d
		elif state==1:
			if(s.counter == 4):
				 state = 2
				 #print 'ok1'
			s.counter+=1
			s.message+=d;
		elif state==2:
			checksum = 0
			for i in range (0,6):
				checksum^=ord(s.message[i])
			if((checksum == c) and ((s.message[1] == chr(0xAF) and mode == 0)or(s.message[1] == chr(0xDF) and mode == 1))):
				state = 3
				s.counter = 0
				dataLength = ord(s.message[5])
				#print ":".join("{:02x}".format(ord(c)) for c in s.message), ord(s.message[5])
				s.message+=d
				#print 'ok2'
			else:
				 state = 0
		elif state==3:
			if(s.counter == dataLength-1):
				 state = 4
				 #print 'ok3'
			s.counter+=1
			s.message+=d
		elif state==4:
			totalChecksum=0;
			for i in range (0,s.pointer):
				totalChecksum^=ord(s.message[i])
			if(totalChecksum == c):
				totalLength = s.pointer+1
				s.message+=d
				s.message+=chr(0x00)
				ret=1
				s.typ = ord(s.message[2])>>4
				s.vendor = ord(s.message[2])&0x0F
				s.model = ord(s.message[3])>>4
				s.address = ord(s.message[3])&0x0F
				command = s.message[7]
				#print 'ok4'
			state = 0
		s.pointer+=1
	print s.pointer, s.counter, dataLength, totalLength, s.typ , s.address, s.vendor, s.model, s.command
	return ret, s.pointer, s.counter, dataLength, totalLength, s.typ , s.address, s.vendor, s.model, command
		
	
def executeCheckMessage(s, u, genericList, genericListLength, specialList, specialListLength):
	if((s.typ == u.typ) and ((s.address == u.address)or(s.address == BROADCAST))):	
		for i in range (0,genericListLength):
			if(genericList[i]==s.command):
				return 1
		for i in range (0,specialListLength):
			if(specialList[i]==s.command and ((s.vendor == u.vendor) and (s.model == u.model))):
				return 1
	return 0

def createMessage(u,s,payload, payloadLen, c):
	buf=''
	checksum=0
	buf+=chr(0xFD)
	buf+=chr(0xDF)
	buf+=chr(u.typ<<4 | (u.vendor & 0x0F))
	buf+=chr(u.model<<4 | (u.address & 0x0F))
	buf+=chr(0x00)
	buf+=chr(payloadLen + s.dataLength +3)
	for i in range(0,6):
		checksum^=ord(buf[i])
	buf+=chr(checksum)
	for i in range(0,s.dataLength):
		buf+=s.message[7+i]
	msb = ((c>>8) & 0xFF)
	lsb = (c & 0xFF)	
	buf+=chr(msb)
	buf+=chr(lsb)
	buf+=chr(payloadLen)
	for i in range(0,payloadLen):
		buf+=payload[i];
	checksum=0
	for i in range(0,10+s.dataLength+payloadLen):
		checksum^=ord(buf[i])
	buf+=chr(checksum)
	return 11+s.dataLength+payloadLen,buf


def createRequestMessage( typ,  address, payload,payloadLen):
	messageToSend=''
	checksum=0;
	messageToSend+=chr(0xFA)
	messageToSend+=chr(0xAF)
	messageToSend+=chr(typ<<4 | (0x00 & 0x0F))
	messageToSend+=chr(0x00<<4 | (address & 0x0F))
	messageToSend+=chr(0x00)
	messageToSend+=chr(payloadLen)
	for i in range(0,6):
		checksum^=ord(messageToSend[i])
	messageToSend+=chr(checksum)
	checksum=0
	for i in range(0,payloadLen):
		messageToSend+=payload[i]
		checksum^=ord(payload[i])
	messageToSend+=chr(checksum)
	return 8+payloadLen, messageToSend
