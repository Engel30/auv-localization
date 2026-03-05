import utm

def checksum(message):
	checksum=0
	for i in range(0,len(message)):
		checksum^=ord(message[i])
	checksum=chr(checksum)
	return checksum
	
def DS_POS(ora,xx,yy,zone_number,zone_letter,depth,roll,pitch,yaw,temperature):
	(lat,lon)=utm.to_latlon(xx,yy,zone_number,zone_letter)
	data=str(ora)+','+str(lat)+',N,'+str(lon)+',E,'+str(depth)+',M,'+str(pitch)+',DP,'+str(roll)+',DR,'+str(yaw)+',DY,'+str(temperature)+',DT'
	message='#DS_POS,'+data+'*'
	chk=checksum(message)
	message+=chk
	#print message
	return message
	
def DS_HEAL(ora,glycemia,breath_rate):
	data=str(ora)+','+str(glycemia)+',GL,'+str(breath_rate)+',BR'
	message="#DS_HEAL,"+data+"*"
	chk=checksum(message)
	message+=chk
	return message

def DS_DEC(ora,depth1,minutes):
	data=str(time)+','+str(depth1)+','+str(minutes)+',M'
	message="#DS_DEC,"+data+"*"
	chk=checksum(message)
	message+=chk
	return message

def DS_MES(ora,data):
	message="#DS_DEC,"+data+"*"
	chk=checksum(message)
	message+=chk
	return message
