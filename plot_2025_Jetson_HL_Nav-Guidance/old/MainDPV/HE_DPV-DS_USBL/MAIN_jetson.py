from lib import xsens, openups2, KF, ms5837, diveMB
from lib.utilities import *
from lib.Message import *
from lib.RMatrix import *
from lib.USBL import *
from lib.sdt import *
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

#GUARDARE COME ABILITARE HEAL, SOLUZIONE DIFFERENZIARE HEAL HEALS

debug=False #ABILITA i LOG
l=len([name for name in os.listdir('.') if os.path.isfile(name)])
Scrittura=False #semafore invio messaggio su USBL
Heal_enabled=False #invio heal
remote_address='2'#indirizzo dell'altro modem usbl
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
evologics=Usbl("192.168.0.138",9200)
l=len([name for name in os.listdir('.') if os.path.isfile(name)])
#map_queue = Queue.Queue()
pos_queue = Queue.Queue()# coda in cui vengono inserite le misurazioni dell'USBL
ack_queue = Queue.Queue()# al momento non utilizzata
mes_queue = Queue.Queue()
date_setted=False
f_tcp=0

def tcp_server(): #Comandi dal tablet DONT TOUCH
	TCPserverstate = stateMachine()
	mb=unit()
	mb.type=0
	mb.address=1
	mb.vendor=0
	mb.model=1
	genericList=['\0','\1','\2']
	genericListLength = 3
	specificList= []
	specificListLength = 0
	bind_ip = '0.0.0.0'
	bind_port = int(sys.argv[1])
	server_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_tcp.bind((bind_ip, bind_port))
	server_tcp.listen(5)  # max backlog of connections
	print 'Listening on {}:{}'.format(bind_ip, bind_port)
	def handle_client_connection(client_socket):
		request = client_socket.recv(1024)
		if debug==True:
			aaa=":".join("{:02x}".format(ord(c)) for c in request)
		print ":".join("{:02x}".format(ord(c)) for c in request)
		[c,TCPserverstate.pointer,TCPserverstate.counter,TCPserverstate.dataLength, TCPserverstate.totalLength,TCPserverstate.typ , TCPserverstate.address,TCPserverstate.vendor,TCPserverstate.model,TCPserverstate.command]=executeStateMachine(request,TCPserverstate,0)
		if c>0 and executeCheckMessage(TCPserverstate,mb,genericList,genericListLength,specificList,specificListLength)==1 and TCPserverstate.address!=0xF:
			#print  ":".join("{:02x}".format(ord(c)) for c in TCPserverstate.message)
			b="\0" * 2
			[s,ack]=createMessage(mb,TCPserverstate,b,0,0)
			#print ":".join("{:02x}".format(ord(c)) for c in ack)
			client_socket.send(ack)
			global udp_state
			if debug==True:
				global f_tcp
				f_tcp = file("tcp"+str(l)+".txt", 'a')# 
				f_tcp.write(ORA()+'\t'+aaa+'\r\n')
				f_tcp.close()
			if (TCPserverstate.command=='\0'):# In questo momento il tablet invia solo start e stop
				udp_state=False
				print "stop"
			elif (TCPserverstate.command=='\1'):
				udp_state=True
				print "start"
				global date_setted
				if date_setted==False:
					hh= request[9:11]
					mm=request[11:13]
					ss=request[13:15]
					time_tuple = ( 2020, # Year
						4, # Month
						9, # Day
						int(hh), # Hour
						int(mm), # Minute
						int(ss), # Second
						0, # Millisecond
					)
					date_setted=True
					linux_set_time(time_tuple)
			elif (TCPserverstate.command=='\2'): #DA IMPLEMENTARE
				mes_queue.put(request[9:9+TCPserverstate.dataLength-2])		
				#msg=request[9:9+ord(TCPserverstate.dataLength)-2]
				#print msg
				#mes_queue.put('1;'+msg)
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
def udp_client(): #DONT TOUCH
	while tcp_enabled:
		time.sleep(0.5)		
	serverAddressPort   = (address[0], int(sys.argv[2]))
	bufferSize          = 1024
	# Create a UDP socket at client side
	UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
	while True:
		#STREAMING VIA UDP DELLA POSIZIONE
		if (udp_state): 
			# i=(i+1)%n_lines
			# bytesToSend = lines[i].rstrip('\n')
			global FIRST_POS, x, y, z, roll, pitch, yaw, zone_letter, zone_number, temperature, x_utm, y_utm
			if FIRST_POS:
				dspos = DS_POS(x_utm+x,y_utm+y,zone_number,zone_letter,z,roll,pitch,yaw,temperature) # Sistemare la creazione della stringa
				print dspos

				UDPClientSocket.sendto(dspos, serverAddressPort)#Invio dati
				dsheal= DS_HEAL(diveMB.gl,diveMB.br,diveMB.hr)
				dsdec= DS_DEC(diveMB.dec)
				print dsheal
				UDPClientSocket.sendto(dsheal, serverAddressPort)#Invio dati
				UDPClientSocket.sendto(dsdec, serverAddressPort)#Invio dati
		global surf_message
		if (surf_message!="\0"): # SE DEVO INVIARE UN MESSAGGIO DALLA SUPERFICIE LO STREAMMO AL TABLET
			#ack="\0"
			#while (ack!="#DS_MES"):
			message2=surf_message+"*"
			chk=checksum(message2)
			message2+=chk
			UDPClientSocket.sendto(message2, serverAddressPort)
			ready = select.select([UDPClientSocket], [], [], 0.1)
			if ready[0]:
				print "ready"
				bytesAddressPair = UDPClientSocket.recvfrom(30)
				ack = bytesAddressPair[0]
				if (ack==surf_message):
					print ack
					surf_message="\0"
		time.sleep(1)
			
def USBL_read_loop():
	while True:
		data=evologics.read_data()
		if data:
			if debug==True:
				f_usbl = file("usbl"+str(l)+".txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z ax ay az
				f_usbl.write(ORA()+"\t"+data+'\r\n')
				f_usbl.close()
			campi=evologics.parse_msg()
			print campi
			# if campi[0]=="USBLLONG":
				# global x, y, z, roll, pitch, yaw, FIRST_POS_Scouter, pos_queue
				# FIRST_POS_Scouter=True
				# x=float(campi[4])
				# y=float(campi[5])
				# z=float(campi[6]) # Ho il profondimetro
				# #Li prendo dalla IMU
				# # roll=float(campi[10])
				# # pitch=float(campi[11])
				# # yaw=float(campi[12])
				# rssi=campi[14]
				# accuracy=campi[16]
				# #pos_queue.put([x,y,z,roll,pitch,yaw])
				# pos_queue.put([x,y,z])#la devo inviare al filtro che salva lo stato del sistema in una variabile di stato
			# elif campi[0]=="USBLANGLES":
				# roll=campi[8]
				# pitch=campi[9]
				# yaw=campi[10]
				# rssi=campi[11]
				# accuracy=campi[13]
			if campi[0]=="RECVIM":
				wgs84=campi[10].split(';')
				print wgs84
				if wgs84[0]=='0':
					global latitude, longitude, x_utm, y_utm, zone_letter,zone_number, FIRST_POS
					FIRST_POS=True #Ricevuta prima posizione scooter
					latitude=float(wgs84[1])
					wg=wgs84[2].split('\r')
					longitude=float(wg[0])
					(x_utm,y_utm,zone_number,zone_letter)=utm.from_latlon(latitude,longitude)
				elif wgs84[0]=='1':
					global surf_message
					wg=wgs84[1].split('\r')
					surf_message="#DS_MES,"+wg[0]
					#print surf_message
				elif wgs84[0]=='2':	# RICEZIONE ENU
					global FIRST_POS_Scouter, pos_queue
					FIRST_POS_Scouter=True
					xx=float(wgs84[1])
					wg=wgs84[2].split('\r')
					yy=float(wg[0])
					if FIRST_POS:
						pos_queue.put([xx,yy])
				elif wgs84[0][0:3]=='POS': #Richesta Posizione VERIFICARE SE CI SARa' \R
					(lat,lon)=utm.to_latlon(x_utm+x,y_utm+y,zone_number,zone_letter)
					msg='0;'+str(lat)+';'+str(lon)+';'+str(z)
					IM=INSTANT_MSG("0",remote_address,"ack",len(msg),msg,True)
					print IM.message
					if debug==True:
						f_usbl = file("usbl"+str(l)+".txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z ax ay az
						f_usbl.write(ORA()+"\t"+IM.message+'\r\n')
						f_usbl.close()
					evologics.write_data(IM.message)
				elif wgs84[0][0:5]=='HEALS':
					global Heal_enabled
					Heal_enabled=True
				elif wgs84[0][0:4]=='HEAL': #Richesta Posizione VERIFICARE SE CI SARa' \R
					#msg='2;'+str(diveMB.gl)+';'+str+';'+z
					msg='3;'+str(diveMB.gl)+';'+str(diveMB.br)+';'+str(diveMB.hr)
					IM=INSTANT_MSG("0",remote_address,"ack",len(msg),msg,True)
					print IM.message
					if debug==True:
						f_usbl = file("usbl"+str(l)+".txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z ax ay az
						f_usbl.write(ORA()+"\t"+IM.message+'\r\n')
						f_usbl.close()
					evologics.write_data(IM.message)
				elif wgs84[0][0:6]=='NOHEAL':
					global Heal_enabled
					Heal_enabled=False					
			elif campi[0]=="DELIVEREDIM":
				c=campi[1].split('\r')
				ack_queue.put(c[0])	
			
		
def USBL_write_loop(): #DA VEDERE 
	global Scrittura #SEMAFORO SCRITTURA
	while True:
		if not mes_queue.empty() and not Scrittura:
			Scrittura=True
			msg="1;"+mes_queue.get()
			IM=INSTANT_MSG("0",remote_address,"ack",len(msg),msg,True)
			print IM.message
			if debug==True:
				f_usbl = file("usbl"+str(l)+".txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z ax ay az
				f_usbl.write(ORA()+"\t"+IM.message+'\r\n')
				f_usbl.close()
			evologics.write_data(IM.message)
			now=time.time()
			expired_time=0;
			while expired_time<2 and ack_queue.empty():
				expired_time=time.time()-now
				time.sleep(0.01)
			a='\0'
			if not ack_queue.empty():
				a=ack_queue.get()
			while a!=remote_address:	
				evologics.write_data(IM.message)
				now=time.time()
				expired_time=0;
				while expired_time<2 and ack_queue.empty():
					expired_time=time.time()-now
					time.sleep(0.01)
				a='\0'
				if not ack_queue.empty():
					a=ack_queue.get()	
			
			
		time.sleep(0.5)
		Scrittura=False
#DEVO INVIARE IN SUPERFICIE LA SALUTE
#INVIARE EVENTUALI MESSAGGI IN SUPERFICIE
#EVENTUALMENTE INVIARE LA POSIZIONE FILTRATA	
def USBL_write_heal_loop(): #DA VEDERE 
	global Scrittura, Heal_enabled #SEMAFORO SCRITTURA
	while True:
		if Heal_enabled and not Scrittura:
			Scrittura=True
			msg='3;'+str(diveMB.gl)+';'+str(diveMB.br)+';'+str(diveMB.hr)
			IM=INSTANT_MSG("0",remote_address,"ack",len(msg),msg,True)
			print IM.message
			if debug==True:
				f_usbl = file("usbl"+str(l)+".txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z ax ay az
				f_usbl.write(ORA()+"\t"+IM.message+'\r\n')
				f_usbl.close()
			evologics.write_data(IM.message)
			time.sleep(2)
			Scrittura=False
		time.sleep(28)
		

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
		    #map_queue.put([longitude,latitude])#longitudine e latitudine vengono inviate su map_queue
		    #print utm.from_latlon(latitude, longitude)[0]
		    
def fusion(): # SISTEMARE CON LE NUOVE POSIZIONI, PROVARE A MODELLARE IL RITARDO!
	global pos_queue, zone_letter, zone_number, roll, pitch, yaw, x, y, z, temperature
	while pos_queue.empty():
		time.sleep(0.1)
	#(x_utm,y_utm,zone_number,zone_letter)=utm.from_latlon(posi[1],posi[0])
	ahrs=xsens.MTI670('/dev/ttyUSB0')#AHRS
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
	Q=np.identity(3)*0.1
	R1=np.identity(3)*1
	R2=np.identity(6)*1
	R2[3:6,3:6]=R2[3:6,3:6]*250
	C1=np.matrix('0, 0, 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1')
	C2=np.matrix('0, 0, 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1; 1, 0, 0, 0, 0, 0, 0, 0, 0; 0, 1, 0, 0, 0, 0, 0, 0, 0; 0, 0, 1, 0, 0, 0, 0, 0, 0')
	d=KF.KF(Q,R1,R2,C1,C2)
	posi=pos_queue.get()#NON LO DEVO PIU' RUOTARE PERCHE' IN ENU		
	d.setX(np.matrix([[posi[0]],[posi[1]],[-sensor.depth()],[0],[0],[0],[0],[0],[0]]))#settare la condizione iniziale del filtro	
	xx=d.getX()
	x=xx[0,0]
	y=xx[1,0]
	z=xx[2,0]
	if debug==True:
		f_kalman = file("kalman"+str(l)+".txt", 'a')# millis angoli acc x_usbl y_usbl z_usbl depth x y z temperature fax fay faz
		f_kalman.write('{0}	{1}	{2}	{3}	'.format(time.time(),roll,pitch,yaw))
		f_kalman.write('{0}	{1}	{2}	'.format(0,0,0))
		f_kalman.write('{0}	{1}	{2}'.format(posi[0],posi[1],sensor.depth()))
		f_kalman.write('{0}	{1}	{2}	{3}	'.format(posi[0],posi[1],sensor.depth(),sensor.temperature()))
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
		#print roll
		pitch=ahrs.getEul()[1]
		yaw=ahrs.getEul()[2]
		if debug==True:
			f_kalman.write('{0}	{1}	{2}	{3}	'.format(time.time(),roll,pitch,yaw))
			f_kalman.write('{0}	{1}	{2}	'.format(acc[0,0],acc[1,0],acc[2,0]))
		if not pos_queue.empty():#cambiare con pos 
			posi=pos_queue.get()		
			yy=np.matrix([[facc[0,0]],[facc[1,0]],[facc[2,0]],[posi[0]],[posi[1]],[-sensor.depth()]])
			d.update(yy,math.radians(roll),math.radians(pitch),math.radians(yaw),False)
			if debug==True:
				f_kalman.write('{0}	{1}	{2}'.format(posi[0],posi[1],sensor.depth()))
		else:
			yy=np.matrix([[facc[0,0]],[facc[1,0]],[facc[2,0]]])
			d.update(yy,math.radians(roll),math.radians(pitch),math.radians(yaw),True)
			if debug==True:
				f_kalman.write('{0}	{1}	{2}'.format(0,0,0))
		xx=d.getX()
		x=xx[0,0]
		y=xx[1,0]
		z=xx[2,0]
		if debug==True:
			f_kalman.write('{0}	{1}	{2}	{3}	'.format(x,y,z,temperature))
			f_kalman.write('{0}	{1}	{2}	'.format(xx[6,0],xx[7,0],xx[8,0]))
			f_kalman.write('{0}	{1}	{2}	'.format(ahrs.getGyro()[0],ahrs.getGyro()[1],ahrs.getGyro()[2]))
			f_kalman.write('{0}	{1}	{2}	'.format(ahrs.getMag()[0],ahrs.getMag()[1],ahrs.getMag()[2]))
			f_kalman.write("\r\n")
		time.sleep(0.2)
	f_kalman.close()


def main():
	if len(sys.argv)==4:
		if sys.argv[3]=='debug':
			global debug
			debug=True
	
	if len(sys.argv)>2:
		tcp = threading.Thread(name='tcp_server', target=tcp_server)
		tcp.daemon = True
		tcp.start()
		
		dive_TCP = threading.Thread(name='tcp_server', target=diveMB.dive_MB)
		dive_TCP.daemon = True
		dive_TCP.start()
		
		udp = threading.Thread(name='udp_client', target=udp_client)
		udp.daemon = True
		udp.start()
		
		usbl_read = threading.Thread(name='usbl_read', target=USBL_read_loop)
		usbl_read.daemon = True
		usbl_read.start()
		
		while not FIRST_POS or not FIRST_POS_Scouter: 
			time.sleep(1)
				
		kalman_filter = threading.Thread(name='usbl_read', target=fusion)
		kalman_filter.daemon = True
		kalman_filter.start()	
		
		usbl_heal = threading.Thread(name='usbl_read', target=USBL_write_heal_loop)
		usbl_heal.daemon = True
		usbl_heal.start()	


		# gps = threading.Thread(name='gps_udp', target=GPS_loop)
		# gps.daemon = True
		# gps.start()
		
		USBL_write_loop()
		
	else:
		print "Few Arguments. Insert TCP, UDP ports "
			# charge=ups.read_charge()
			# print charge	

if __name__== '__main__':main()
