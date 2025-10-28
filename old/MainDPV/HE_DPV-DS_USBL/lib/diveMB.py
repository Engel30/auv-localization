import socket
import threading
import numpy as np

hr=0
br=0
gl=0.0
dec = np.empty((0,2), int)


def parse_dive_msg(msg):
	global hr, br,gl,dec
	campi=msg.split(',')
	if campi[0]=='#DS_HEAL':
		hr=int(campi[7])
		br=int(campi[5])
		gl=float(campi[3])
		return 0
	elif campi[0]=='#DS_DEC':
		dec = np.empty((0,2), int)
		for i in range((len(campi)-4)/4):
			dec = np.append(dec, np.array([[campi[2*i+3],campi[2*i+5]]]), axis=0)
		return 1
	return 9


def dive_MB():
	bind_ip = '0.0.0.0'
	bind_port = 1234
	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server.bind((bind_ip, bind_port))
	server.listen(2)  # max backlog of connections
	print 'Listening on {}:{}'.format(bind_ip, bind_port)
	
	
	def handle_client_connection(client_socket):
		request = client_socket.recv(1024)
		print request
		if parse_dive_msg( request)==0:
			print hr, br,gl
		else:
			print dec
		#if TCPserverstate.address!=0xF:	
		#client_socket.send('ACK!')
		client_socket.close()
	
	while True:
	    client_sock, address = server.accept()
	   # print 'Accepted connection from {}:{}'.format(address[0], address[1])
	    client_handler = threading.Thread(
	        target=handle_client_connection,
	        args=(client_sock,)  # without comma you'd get a... TypeError: handle_client_connection() argument after * must be a sequence, not _socketobject
	    )
	    client_handler.start()
	

