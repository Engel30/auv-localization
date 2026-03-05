
import time, serial, struct

def fusion(b1,b2,b3,b4):
	list1=[b1,b2,b3,b4]
	aa= bytearray(list1) 
	return struct.unpack('<f', aa)[0]
	

class MTI670(object):
	_eul=[0,0,0]
	_acc=[0,0,0]
	_gyro=[0,0,0]
	_facc=[0,0,0]
	_mag=[0,0,0]
	def __init__(self,PORT):
		self._ser = serial.Serial(PORT, 460800,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,xonxoff=False, rtscts=False, dsrdtr=False)
    
	def read_data(self):
		bytesToRead = self._ser.inWaiting()
		if bytesToRead>100:
			self._ser.read(bytesToRead)
		bytesToRead=0
		while bytesToRead <99:
			bytesToRead = self._ser.inWaiting()
			time.sleep(0.001)
		data=self._ser.read(bytesToRead)
		self._arr=bytearray()
		for i in data:
			self._arr.append(i)
		#print ":".join("{:02x}".format(ord(c)) for c in data)
		#print "\n"
		self._eul=[fusion(self._arr[22],self._arr[21],self._arr[20],self._arr[19]),fusion(self._arr[26],self._arr[25],self._arr[24],self._arr[23]),fusion(self._arr[30],self._arr[29],self._arr[28],self._arr[27])]
		self._acc=[fusion(self._arr[37],self._arr[36],self._arr[35],self._arr[34]),fusion(self._arr[41],self._arr[40],self._arr[39],self._arr[38]),fusion(self._arr[45],self._arr[44],self._arr[43],self._arr[42])]
		self._gyro=[fusion(self._arr[67],self._arr[66],self._arr[65],self._arr[64]),fusion(self._arr[71],self._arr[70],self._arr[69],self._arr[68]),fusion(self._arr[75],self._arr[74],self._arr[73],self._arr[72])]
		self._mag=[fusion(self._arr[82],self._arr[81],self._arr[80],self._arr[79]),fusion(self._arr[86],self._arr[85],self._arr[84],self._arr[83]),fusion(self._arr[90],self._arr[89],self._arr[88],self._arr[87])]
		self._facc=[fusion(self._arr[52],self._arr[51],self._arr[50],self._arr[49]),fusion(self._arr[56],self._arr[55],self._arr[54],self._arr[53]),fusion(self._arr[60],self._arr[59],self._arr[58],self._arr[57])]

        
	def getEul(self):
		return self._eul
	
	def getAcc(self):
		return self._acc
    
	def getGyro(self):
		return self._gyro
    
	def getMag(self):
		return self._mag

	def getFacc(self):
		return self._facc
		

# ahrs=MTI670('/dev/ttyUSB0')


# while 1:
	# ahrs.read_data()
	# print "roll",ahrs.getEul()[0],"pitch",ahrs.getEul()[1],"yaw",ahrs.getEul()[2]
	# print "accx",ahrs.getAcc()[0],"accy",ahrs.getAcc()[1],"accz",ahrs.getAcc()[2]
	# time.sleep(0.2)

