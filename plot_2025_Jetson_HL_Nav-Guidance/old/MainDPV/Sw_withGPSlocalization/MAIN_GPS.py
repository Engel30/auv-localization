from lib import xsens, openups2, KF, ms5837 
import os
import numpy as np
import math
import time
import socket
import threading
import pynmea2
import utm
import Queue
import socket
import sys
import select
import threading
from lib.utilities import *
from lib.Message import *
from lib.RMatrix import *

tcp_enabled=True #variabile che indica arrivato il messaggio di start dal tablet
udp_state=False
address= [];
message="\0"
latitude=0
longitude=0
FIRST_POS=False
map_queue = Queue.Queue()
zone_letter=""
zone_number=0
roll=0
pitch=0
yaw=0
x=0
y=0
z=0
temperature=0

def tcp_server():
	TCPserverstate = stateMachine()
	TCPserverstate.state=0
	mb=unit()
	mb.type='\0';
	mb.address='\1';
	mb.vendor='\0';
	mb.model='\1';
	genericList="\0\1"
	genericListLength = 2
	specificList= ""
	specificListLength = 0
	bind_ip = '0.0.0.0'
	bind_port = int(sys.argv[1])
	server_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_tcp.bind((bind_ip, bind_port))
	server_tcp.listen(5)  # max backlog of connections
	print 'Listening on {}:{}'.format(bind_ip, bind_port)
	
	def handle_client_connection(client_socket):
		request = client_socket.recv(1024)
		print ":".join("{:02x}".format(ord(c)) for c in request)
		for i in range(0,len(request)):
			c=executeStateMachine(request[i],ctypes.pointer(TCPserverstate),'\0')
		if c>0 and executeCheckMessage(ctypes.pointer(TCPserverstate),ctypes.pointer(mb),genericList,genericListLength,specificList,specificListLength)==1 and TCPserverstate.address!=chr(0xF):
			ack="\0" * 13
			b="\0" * 2
			createMessage(ack,ctypes.pointer(mb),ctypes.pointer(TCPserverstate),b,0,0)
			#print ":".join("{:02x}".format(ord(c)) for c in ack)
			client_socket.send(ack)
			global udp_state
			if (TCPserverstate.command=='\0'):# In questo momento il tablet invia solo start e stop
				udp_state=False
				print "stop"
			elif (TCPserverstate.command=='\1'):
				udp_state=True
				print "start"
		client_socket.close()
	
	while True:
		global address
		client_sock, address = server_tcp.accept()
		print 'Accepted connection from {}:{}'.format(address[0], address[1])
		global tcp_enabled
		tcp_enabled=False
		client_handler = threading.Thread(
	        target=handle_client_connection,
	        args=(client_sock,)  # without comma you'd get a... TypeError: handle_client_connection() argument after * must be a sequence, not _socketobject
		)
		client_handler.start()

# questo e' il client che invia la posizione al tablet, stringhe create dalle funzioni della libreria Message
def udp_client():
	while tcp_enabled:
		time.sleep(0.5)		
	serverAddressPort   = (address[0], int(sys.argv[2]))
	bufferSize          = 1024
	# Create a UDP socket at client side
	UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
	while True:
		# Send to server using created UDP socket
		if (udp_state):
			# i=(i+1)%n_lines
			# bytesToSend = lines[i].rstrip('\n')
			global FIRST_POS, x, y, z, roll, pitch, yaw, zone_letter, zone_number, temperature
			if FIRST_POS:
				#print y
				dspos = DS_POS(time.time(),x,y,zone_number,zone_letter,z,roll,pitch,yaw,temperature)
				UDPClientSocket.sendto(dspos, serverAddressPort)#Invio dati
		global message
		if (message!="\0"):
			#ack="\0"
			#while (ack!="#DS_MES"):
			UDPClientSocket.sendto(message, serverAddressPort)
			ready = select.select([UDPClientSocket], [], [], 0.1)
			if ready[0]:
				print "ready"
				bytesAddressPair = UDPClientSocket.recvfrom(30)
				ack = bytesAddressPair[0]
				if (ack=="#DS_MES"):
					print ack
					message="\0"
		time.sleep(1)

def GPS_loop():#LOOP per la ricezione del GPS da UDP client
	localIP     = "127.0.0.1"
	localPort   = 8881
	bufferSize  = 1024
	 
	# Create a datagram socket
	UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
	
	# Bind to address and ip
	UDPServerSocket.bind((localIP, localPort))
	print("UDP server up and listening")
	
	# Listen for incoming datagrams
	while(True):
		bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
		message = bytesAddressPair[0]
		try:
			msg = pynmea2.parse(message)
			if (msg.lat!="" and msg.lon!=""):
				global latitude, longitude, FIRST_POS
				latitude=float(msg.lat[:2])+float(msg.lat[2:])/60
				longitude=float(msg.lon[:3])+float(msg.lon[3:])/60
				FIRST_POS=True
				#map_queue.put([longitude,latitude,random.randint(0,2)])
				map_queue.put([longitude,latitude])#longitudine e latitudine vengono inviate su map_queue
		except:
			print "parsing error"
			    #print utm.from_latlon(latitude, longitude)[0]
			
		    
def fusion():
	global map_queue, zone_letter, zone_number, roll, pitch, yaw, x, y, z, temperature
	while map_queue.empty():
		time.sleep(0.1)
	posi=map_queue.get()
	(x_utm,y_utm,zone_number,zone_letter)=utm.from_latlon(posi[1],posi[0])
	ahrs=xsens.MTI3()#AHRS
	#PROFONDIMETRO
	sensor = ms5837.MS5837_30BA(1) # Default I2C bus is 1 (jetson tx2 auvidea j120)
	if not sensor.init():
		print "Sensor could not be initialized"	
	if not sensor.read():
		print "Sensor read failed!"
	sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)

	Q=np.identity(3)*0.5
	R1=np.identity(3)*0.5
	R2=np.identity(6)*0.5
	R2[3:6,3:6]=R2[3:6,3:6]*50
	C1=np.matrix('0, 0, 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1')
	C2=np.matrix('0, 0, 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1; 1, 0, 0, 0, 0, 0, 0, 0, 0; 0, 1, 0, 0, 0, 0, 0, 0, 0; 0, 0, 1, 0, 0, 0, 0, 0, 0')
	d=KF.KF(Q,R1,R2,C1,C2)
	l=len([name for name in os.listdir('.') if os.path.isfile(name)])
	f_kalman = file("kalman"+str(l)+".txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z temperature ax ay az
	d.setX(np.matrix([[x_utm],[y_utm],[sensor.depth()],[0],[0],[0],[0],[0],[0]]))
	f_kalman.write('{0}	{1}	{2}	{3}	'.format(time.time(),0,0,0))
	f_kalman.write('{0}	{1}	{2}	'.format(0,0,0))
	f_kalman.write('{0}	{1}	{2}	'.format(posi[0],posi[1],sensor.depth()))
	f_kalman.write('{0}	{1}	{2}	{3}	'.format(x_utm,y_utm,sensor.depth(),sensor.temperature()))
	f_kalman.write('{0}	{1}	{2}	'.format(0,0,0))
	f_kalman.write("\r\n")
	while 1:
		ahrs.read_data()
		sensor.read()
		temperature=sensor.temperature()
		# if sensor.read():
			# print("D: %0.1f m \tT: %0.2f C") % (
			# sensor.depth(), #dovrebbe essere in metri
			# sensor.temperature()), # Default is degrees C (no arguments)
		acc=np.matrix([[ahrs.getAcc()[0]],[ahrs.getAcc()[1]],[ahrs.getAcc()[2]]])
		g=np.matrix([[0],[0],[9.814]])
		facc=acc-Rxy(-math.radians(ahrs.getEul()[0]),-math.radians(ahrs.getEul()[1]))*g#DA VERIFICARE
		roll=ahrs.getEul()[0]
		pitch=ahrs.getEul()[1]
		yaw=ahrs.getEul()[2]
		f_kalman.write('{0}	{1}	{2}	{3}	'.format(time.time(),roll,pitch,yaw))
		f_kalman.write('{0}	{1}	{2}	'.format(acc[0,0],acc[1,0],acc[2,0]))
		if not map_queue.empty():
			posi=map_queue.get()
			(x_utm,y_utm,zone_number,zone_letter)=utm.from_latlon(posi[1],posi[0])
			#print [x_utm,y_utm]
			yy=np.matrix([[facc[0,0]],[facc[1,0]],[facc[2,0]],[x_utm],[y_utm],[sensor.depth()]])
			d.update(yy,math.radians(roll),math.radians(pitch),math.radians(yaw),False)
			f_kalman.write('{0}	{1}	{2}	'.format(posi[0],posi[1],sensor.depth()))
		else:
			yy=np.matrix([[facc[0,0]],[facc[1,0]],[facc[2,0]]])
			d.update(yy,math.radians(roll),math.radians(pitch),math.radians(yaw),True)
			f_kalman.write('{0}	{1}	{2}	'.format(0,0,0))
		xx=d.getX()
		x=xx[0,0]
		y=xx[1,0]
		z=xx[2,0]
		f_kalman.write('{0}	{1}	{2}	{3}	'.format(x,y,z,temperature))
		f_kalman.write('{0}	{1}	{2}	'.format(xx[6,0],xx[7,0],xx[8,0]))
		f_kalman.write('{0}	{1}	{2}	'.format(ahrs.getGyro()[0],ahrs.getGyro()[1],ahrs.getGyro()[2]))
		f_kalman.write('{0}	{1}	{2}	'.format(ahrs.getMag()[0],ahrs.getMag()[1],ahrs.getMag()[2]))
		f_kalman.write("\r\n")
		
		#print "roll",roll,"pitch",pitch,"yaw",yaw
		#print "accx",ahrs.getFacc()[0],"accy",ahrs.getFacc()[1],"accz",ahrs.getFacc()[2]

		#facc=Rzyx(math.radians(ahrs.getEul()[0]),math.radians(ahrs.getEul()[1]),math.radians(ahrs.getEul()[2]))*acc-g
		#print facc
		# time.sleep(0.05)

		time.sleep(0.15)
	f_kalman.close()

def main():
	if len(sys.argv)>2:
		tcp = threading.Thread(name='tcp_server', target=tcp_server)
		tcp.daemon = True
		tcp.start()
		
		udp = threading.Thread(name='udp_client', target=udp_client)
		udp.daemon = True
		udp.start()

		gps = threading.Thread(name='gps_udp', target=GPS_loop)
		gps.daemon = True
		gps.start()
		
		fusion()
		
	else:
		print "Few Arguments. Insert TCP, UDP ports "
			# charge=ups.read_charge()
			# print charge	

if __name__== '__main__':main()
