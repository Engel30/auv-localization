import socket
import select
import threading
import sys
import time

def checksum(message):
	checksum=0
	for i in range(1,len(message)-1):
		checksum^=ord(message[i])
	if checksum >15:
		checksum=hex(checksum)[2:]
	else:
		checksum='0'+hex(checksum)[2:]
	return checksum	
	
class INSTANT_MSG:
	def __init__(self,PID,Dest_ADD,flag,lung,data,ext):
		self.PID=PID
		self.address=Dest_ADD
		self.flag=flag
		self.data=data
		self.lung=lung
		if self.lung<65:
			if ext:
				self.message="AT*SENDIM,p"+PID+","+str(lung)+","+Dest_ADD+","+flag+","+data
			else:
				self.message="AT*SENDIM,"+str(lung)+","+Dest_ADD+","+flag+","+data
		else:
			self.message="ERROR"

class Usbl:
	def __init__(self,host,port):
		self.host=host
		self.port=port
		self.client_telnet = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.client_telnet.settimeout(2)
		try:
			self.client_telnet.connect((host, port))
			print "Connected"
		except:
			print 'Unable to connect'
		time.sleep(0.5)
		self.write_data("+++ATC")
			
	def read_data(self):
		socket_list = [self.client_telnet]	
		# Get the list sockets which are readable
		read_sockets, write_sockets, error_sockets = select.select(socket_list , [], [])

		for sock in read_sockets:
			#incoming message from remote server
			if sock == self.client_telnet:
				data = sock.recv(4096)
				if not data :
					try:
						self.client_telnet.connect((host, port))
					except:
						print 'Unable to connect'
					return "ERROR"
				else:
					self.last_message=data
					return data
	
	def write_data(self,msg):
		if not msg[len(msg)-1]=="\n":
			msg=msg+"\n"
		self.client_telnet.send(msg)	
		
	def create_data_message(self,data):
		message="#DS_MES,"+data+"*"
		chk=checksum(message)
		message+=chk
		return message
		
	def parse_msg(self):
		self.msg_param=self.last_message.split(",")
		return self.msg_param
		# if msg_param[0]=="USBLLONG":
			
		# elif msg_param[0]=="USBLANGLES":
			
		# elif msg_param[0]=="RECVIM":
			
		# elif msg_param[0]=="DELIVEREDIM":
			
		#MANCA GLI OK E I MESSAGGI DI ERRORE
