from lib import xsens, openups2, KF, ms5837 
from lib.utilities import *
from lib.Message import *
from lib.RMatrix import *
from lib.USBL import *
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
import os

remote_address='1'#indirizzo dell'altro modem usbl
tcp_enabled=True #variabile che indica arrivato il messaggio di start dal tablet
udp_state=False
address= [];
surf_message="\0"#messaggio dalla superfice da inviare al tablet
#Wgs84 della base
latitude=0
longitude=0
#arrivata la coordinata della base
FIRST_POS=False
#arrivata la prima misurazione della USBL
FIRST_POS_Scouter=False
#coordinata della base in utm
zone_letter=""
zone_number=0
x_utm=0
y_utm=0
#Stato
roll=0
pitch=0
yaw=0
x=0
y=0
z=0
temperature=0
evologics=Usbl("192.168.1.139",9200)
#map_queue = Queue.Queue()
pos_queue = Queue.Queue()# coda in cui vengono inserite le misurazioni dell'USBL
ack_queue =Queue.Queue()# al momento non utilizzata

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
			global FIRST_POS, x, y, z, roll, pitch, yaw, zone_letter, zone_number, temperature, x_utm, y_utm
			if FIRST_POS:
				dspos = DS_POS(time.time(),x_utm+x,y_utm+y,zone_number,zone_letter,z,roll,pitch,yaw,temperature) # Sistemare la creazione della stringa
				UDPClientSocket.sendto(dspos, serverAddressPort)#Invio dati
		global surf_message
		if (surf_message!="\0"):
			#ack="\0"
			#while (ack!="#DS_MES"):
			UDPClientSocket.sendto(surf_message, serverAddressPort)
			ready = select.select([UDPClientSocket], [], [], 0.1)
			if ready[0]:
				print "ready"
				bytesAddressPair = UDPClientSocket.recvfrom(30)
				ack = bytesAddressPair[0]
				if (ack=="#DS_MES"):
					print ack
					surf_message="\0"
		time.sleep(1)
			
def USBL_read_loop():
	f_usbl = file("usbl.txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z ax ay az
	while True:
		data=evologics.read_data()
		if data:
			f_usbl.write(data)
			campi=evologics.parse_msg()
			print campi
			if campi[0]=="USBLLONG":
				global x, y, z, roll, pitch, yaw, FIRST_POS_Scouter, pos_queue
				FIRST_POS_Scouter=True
				x=float(campi[4])
				y=float(campi[5])
				z=float(campi[6]) # Ho il profondimetro
				#Li prendo dalla IMU
				# roll=float(campi[10])
				# pitch=float(campi[11])
				# yaw=float(campi[12])
				rssi=campi[14]
				accuracy=campi[16]
				#pos_queue.put([x,y,z,roll,pitch,yaw])
				pos_queue.put([x,y,z])#la devo inviare al filtro che salva lo stato del sistema in una variabile di stato
			elif campi[0]=="USBLANGLES":
				roll=campi[8]
				pitch=campi[9]
				yaw=campi[10]
				rssi=campi[11]
				accuracy=campi[13]
			elif campi[0]=="RECVIM":
				wgs84=campi[10].split(';')
				print wgs84
				if wgs84[0]=='0':
					global latitude, longitude, x_utm, y_utm, zone_letter,zone_number, FIRST_POS
					FIRST_POS=True
					latitude=float(wgs84[1])
					wg=wgs84[2].split('\r')
					longitude=float(wg[0])
					(x_utm,y_utm,zone_number,zone_letter)=utm.from_latlon(latitude,longitude)
				elif wgs84[0]=='1':
					global surf_message
					surf_message="#DS_MES,"+wgs84[1]+"*"
					chk=checksum(surf_message)
					surf_message+=chk
					#print surf_message
			elif campi[0]=="DELIVEREDIM":
				ack_queue.put(campi[1])	
			
def USBL_write_loop(): # OK
	while True:
		global x, y, z, roll, pitch, yaw, x_utm, y_utm, zone_letter, zone_number
		xx=x_utm-x
		yy=y_utm-y
		(lat,lon)=utm.to_latlon(xx,yy,zone_number,zone_letter)
		msg="0;"+str(lat)+';'+str(lon)+";"+str(z)
		IM=INSTANT_MSG("0",remote_address,"ack",len(msg),msg,True)
		print IM.message
		evologics.write_data(IM.message)
		time.sleep(1)

#LOOP per la ricezione del GPS da UDP client
def GPS_loop():#Non utilizzato
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
	    msg = pynmea2.parse(message)
	    if (msg.lat!="" and msg.lon!=""):
		    global latitude, longitude, FIRST_POS
		    latitude=float(msg.lat[:2])+float(msg.lat[2:])/60
		    longitude=float(msg.lon[:3])+float(msg.lon[3:])/60
		    FIRST_POS=True
		    #map_queue.put([longitude,latitude,random.randint(0,2)])
		    map_queue.put([longitude,latitude])#longitudine e latitudine vengono inviate su map_queue
		    #print utm.from_latlon(latitude, longitude)[0]
		    
def fusion(): # OK
	global pos_queue, zone_letter, zone_number, roll, pitch, yaw, x, y, z, temperature
	while pos_queue.empty():
		time.sleep(0.1)
	#(x_utm,y_utm,zone_number,zone_letter)=utm.from_latlon(posi[1],posi[0])
	ahrs=xsens.MTI3()#AHRS
	#Devo ruotare la posizione per l'ENU
	#PROFONDIMETRO
	sensor = ms5837.MS5837_30BA(1) # Default I2C bus is 1 (jetson tx2 auvidea j120)
	if not sensor.init():
		print "Sensor could not be initialized"	
	if not sensor.read():
		print "Sensor read failed!"
	sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
	ahrs.read_data()
	roll=ahrs.getEul()[0]#Orientatmento iniziale
	pitch=ahrs.getEul()[1]
	yaw=ahrs.getEul()[2]
	Q=np.identity(3)*1
	R1=np.identity(3)*5
	R2=np.identity(6)*0.5
	R2[3:6,3:6]=R2[3:6,3:6]*50
	C1=np.matrix('0, 0, 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1')
	C2=np.matrix('0, 0, 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1; 1, 0, 0, 0, 0, 0, 0, 0, 0; 0, 1, 0, 0, 0, 0, 0, 0, 0; 0, 0, 1, 0, 0, 0, 0, 0, 0')
	d=KF.KF(Q,R1,R2,C1,C2)
	posi=pos_queue.get()
	P_b=np.matrix([[posi[0]],[posi[1]],[posi[2]]]) #Posizione nel body frame
	R=Rzyx(roll,pitch,yaw) # Matrice di rotazione dal Body frame to ENU VERIFICARE LA MATRICE DI ROTAZIONE E LA TERNA DELL'ANTENNA USBL
	P_i=-R*P_b# Posizione nel I frame			
	d.setX(np.matrix([[P_i[0,0]],[P_i[1,0]],[sensor.depth()],[0],[0],[0],[0],[0],[0]]))#settare la condizione iniziale del filtro
	x=xx[0,0] # Posizione iniziale
	y=xx[1,0]
	z=xx[2,0]
	
	l=len([name for name in os.listdir('.') if os.path.isfile(name)])
	f_kalman = file("kalman"+str(l)+".txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z temperature fax fay faz
	f_kalman.write('{0}	{1}	{2}	{3}	'.format(time.time(),roll,pitch,yaw))
	f_kalman.write('{0}	{1}	{2}	'.format(0,0,0))
	f_kalman.write('{0}	{1}	{2}	{3}	'.format(posi[0],posi[1],posi[2],sensor.depth()))
	f_kalman.write('{0}	{1}	{2}	{3}	'.format(P_i[0,0],P_i[1,0],sensor.depth(),sensor.temperature()))
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
		if not map_queue.empty():#cambiare con pos 
			posi=pos_queue.get()
			P_b=np.matrix([[posi[0]],[posi[1]],[posi[2]]]) #Posizione nel body frame
			R=Rzyx(roll,pitch,yaw) # Matrice di rotazione dal Body frame to ENU
			P_i=-R*P_b#Posizione nel I frame			
			yy=np.matrix([[facc[0,0]],[facc[1,0]],[facc[2,0]],[P_i[0,0]],[P_i[1,0]],[sensor.depth()]])
			d.update(yy,math.radians(roll),math.radians(pitch),math.radians(yaw),False)
			f_kalman.write('{0}	{1}	{2}	{3}	'.format(posi[0],posi[1],posi[2],sensor.depth()))
		else:
			yy=np.matrix([[facc[0,0]],[facc[1,0]],[facc[2,0]]])
			d.update(yy,math.radians(roll),math.radians(pitch),math.radians(yaw),True)
			f_kalman.write('{0}	{1}	{2}	{3}	'.format(0,0,0,0))
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

		#facc=Rzyx(math.radians(ahrs.getEul()[0]),math.radians(ahrs.getEul()[1]),math.radians(ahrs.getEul()[2]))*acc-g #uguale facc del 
		#print facc
		# time.sleep(0.05)

		time.sleep(0.2)
	f_kalman.close()


def main():
	if len(sys.argv)>2:
		tcp = threading.Thread(name='tcp_server', target=tcp_server)
		tcp.daemon = True
		tcp.start()
		
		udp = threading.Thread(name='udp_client', target=udp_client)
		udp.daemon = True
		udp.start()
		
		usbl_read = threading.Thread(name='usbl_read', target=USBL_read_loop)
		usbl_read.daemon = True
		usbl_read.start()
		
		while not FIRST_POS:#POSIZIONE INIZIALE TESTA
			time.sleep(1)

		while not FIRST_POS_Scouter:#POSIZIONE INIZIALE SCOUTER
			msg="CONDIZIONE INIZIALE"
			IM=INSTANT_MSG("0",remote_address,"ack",len(msg),msg,True)
			print IM.message
			evologics.write_data(IM.message)
			time.sleep(4)
			
		kalman_filter = threading.Thread(name='usbl_read', target=fusion)
		kalman_filter.daemon = True
		kalman_filter.start()	

		# gps = threading.Thread(name='gps_udp', target=GPS_loop)
		# gps.daemon = True
		# gps.start()
		
		USBL_write_loop()
		
	else:
		print "Few Arguments. Insert TCP, UDP ports "
			# charge=ups.read_charge()
			# print charge	

if __name__== '__main__':main()
