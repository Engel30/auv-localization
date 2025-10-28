import smbus

class openups2(object):
	# Registers
	_UPS_ADDR = 0x38  
	_UPS_VOLT = 0x9
	_UPS_CURR = 0xA
	_UPS_ERR = 0xC
	_UPS_PERCR = 0xD
	_UPS_PERCA = 0xE
	
	def __init__(self,bus=1,tipo="LIFEP04"):
		try:
			self._bus = smbus.SMBus(bus)
			print("openups2 i2c")
		except:
			print("Bus %d is not available.") % bus
			print("Available busses are listed as /dev/i2c*")
			self._bus = None
		
		self.rel_charge=100
		self.abs_charge=100
		self.err=0
		self.current=0
		self.tipo=tipo
		if tipo=="LIFEP04":
			self.voltage=9.6
		elif tipo=="LI-ON":
			self.voltage=11.1
		
	def read_volt(self):
		self.voltage=self._bus.read_word_data(self._UPS_ADDR,self._UPS_VOLT)
		return self.voltage
	
	def read_curr(self):
		self.current=self._bus.read_word_data(self._UPS_ADDR,self._UPS_CURR)
		return self.current

	def read_charge(self):
		self.rel_charge=self._bus.read_word_data(self._UPS_ADDR,self._UPS_PERCR)
		self.abs_charge=self._bus.read_word_data(self._UPS_ADDR,self._UPS_PERCA)
		self.err=self._bus.read_word_data(self._UPS_ADDR,self._UPS_ERR)
		return [self.rel_charge, self.abs_charge, self.err]
