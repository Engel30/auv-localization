import utm
import datetime

def checksum(message):
	checksum=0
	for i in range(1,len(message)-1):
		checksum^=ord(message[i])
	if checksum >15:
		checksum=hex(checksum)[2:]
	else:
		checksum='0'+hex(checksum)[2:]
	return checksum

	
def ORA():
	# hh=now.hour
	# mm=now.minute
	# ss=now.second
	# if hh < 10:
		# HH='0'+str(hh)
	# else:
		# HH=str(hh)	
	# if mm < 10:
		# MM='0'+str(mm)
	# else:
		# MM=str(mm)	
	# if ss < 10:
		# SS='0'+str(ss)
	# else:
		# SS=str(ss)	  
	dateTime = datetime.datetime.now()
	ora = dateTime.strftime("%H%M%S.%f")[0:9]	
	return ora
	
	
def DS_POS(xx,yy,zone_number,zone_letter,depth,roll,pitch,yaw,temperature):
	ora=ORA()
	(lat,lon)=utm.to_latlon(xx,yy,zone_number,zone_letter)
	data=ora+',T,'+str(round(lat,6))+',N,'+str(round(lon,6))+',E,'+str(round(depth,2))+',M,'+str(round(pitch,6))+',DP,'+str(round(roll,6))+',DR,'+str(round(yaw,6))+',DY,'+str(round(temperature,1))+',DT'
	message='#DS_POS,'+data+'*'
	chk=checksum(message)
	message+=chk
	#print message
	return message
	
def DS_HEAL(glycemia,breath_rate,hr):
	ora=ORA()
	data=ora+',T,'+str(round(glycemia,2))+',GL,'+str(breath_rate)+',BR,'+str(hr)+',HR'
	message="#DS_HEAL,"+data+"*"
	chk=checksum(message)
	message+=chk
	return message


def DS_DEC(dec):
	ora=ORA()
	msg=''
	for i in range(len(dec)):
		if float(dec[i][1])>0:
			msg+=','+str(dec[i][0])+',T'+str(i+1)+','+str(dec[i][1])+',DP'+str(i+1)
		else:
			msg+=','+str(dec[i][0])+',T'+str(i)+','+str(dec[i][1])+',DP'+str(i)
	data=ora+',T'+msg+',M'
	message="#DS_DEC,"+data+"*"
	chk=checksum(message)
	message+=chk
	return message
	
def DS_MES(data):
	ora=ORA()
	message="#DS_MES,"+data+"*"
	chk=checksum(message)
	message+=chk
	return message
