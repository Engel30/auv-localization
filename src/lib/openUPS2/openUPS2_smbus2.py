import smbus2

class openups2(object):
	# Registers
	_UPS_ADDR = 0x38  
	_UPS_VOLT = 0x9
	_UPS_CURR = 0xA
	_UPS_ERR = 0xC
	_UPS_PERCR = 0xD
	_UPS_PERCA = 0xE
	
	def __init__(self, bus=1, tipo="LIFEP04"):
		try:
			self._bus = smbus2.SMBus(bus)
			print("openups2 i2c inizializzato (smbus2)")
		except Exception as e:
			print(f"Bus {bus} non disponibile: {e}")
			print("Bus disponibili: /dev/i2c*")
			self._bus = None
		
		self.rel_charge = 100
		self.abs_charge = 100
		self.err = 0
		self.current = 0
		self.tipo = tipo
		if tipo == "LIFEP04":
			self.voltage = 9.6
		elif tipo == "LI-ON":
			self.voltage = 11.1
	
	def read_volt(self):
		if self._bus:
			self.voltage = self._bus.read_word_data(self._UPS_ADDR, self._UPS_VOLT)
		return self.voltage
	
	def read_curr(self):
		if self._bus:
			self.current = self._bus.read_word_data(self._UPS_ADDR, self._UPS_CURR)
		return self.current

	def read_charge(self):
		if self._bus:
			self.rel_charge = self._bus.read_word_data(self._UPS_ADDR, self._UPS_PERCR)
			self.abs_charge = self._bus.read_word_data(self._UPS_ADDR, self._UPS_PERCA)
			self.err = self._bus.read_word_data(self._UPS_ADDR, self._UPS_ERR)
		return [self.rel_charge, self.abs_charge, self.err]
	
	def close(self):
		"""Chiude la connessione I2C"""
		if self._bus:
			self._bus.close()