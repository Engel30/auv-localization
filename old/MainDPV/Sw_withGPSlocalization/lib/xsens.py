
import ctypes
import time

lib = ctypes.cdll.LoadLibrary('./lib/libxsens.so')

class MTI3(object):
	def __init__(self):
		lib.MTI3_new.argtypes = []
		lib.MTI3_new.restype = ctypes.c_void_p
		
		lib.MTI3_init.argtypes = [ctypes.c_void_p]
		lib.MTI3_init.restype = ctypes.c_bool
		
		lib.MTI3_read.argtypes = [ctypes.c_void_p]
		lib.MTI3_read.restype = ctypes.c_void_p
		
		lib.MTI3_eul.argtypes = [ctypes.c_void_p]
		lib.MTI3_eul.restype = ctypes.POINTER(ctypes.c_float)
		
		lib.MTI3_acc.argtypes = [ctypes.c_void_p]
		lib.MTI3_acc.restype = ctypes.POINTER(ctypes.c_float)
		
		lib.MTI3_gyro.argtypes = [ctypes.c_void_p]
		lib.MTI3_gyro.restype = ctypes.POINTER(ctypes.c_float)
		
		lib.MTI3_mag.argtypes = [ctypes.c_void_p]
		lib.MTI3_mag.restype = ctypes.POINTER(ctypes.c_float)
		
		lib.MTI3_facc.argtypes = [ctypes.c_void_p]
		lib.MTI3_facc.restype = ctypes.POINTER(ctypes.c_float)	
			
		self.obj = lib.MTI3_new()
		
		self.init()
		       
	def init(self):
		lib.MTI3_init(self.obj)
		print "xsens i2c ok"
    
	def read_data(self):
		return lib.MTI3_read(self.obj)
        
	def getEul(self):
		return lib.MTI3_eul(self.obj)
	
	def getAcc(self):
		return lib.MTI3_acc(self.obj)
    
	def getGyro(self):
		return lib.MTI3_gyro(self.obj)
    
	def getMag(self):
		return lib.MTI3_mag(self.obj)

	def getFacc(self):
		return lib.MTI3_facc(self.obj)
		

# ahrs=MTI3()
# ahrs.init()

# while 1:
	# ahrs.read_data()
	# print "roll",ahrs.getEul()[0],"pitch",ahrs.getEul()[1],"yaw",ahrs.getEul()[2]
	# print "accx",ahrs.getAcc()[0],"accy",ahrs.getAcc()[1],"accz",ahrs.getAcc()[2]
	# time.sleep(0.05)

