import ms5837
import xsens
import openups2
import time
from RMatrix import *
import numpy as np
import math

#sensor = ms5837.MS5837_30BA(1) # Default I2C bus is 1 (Raspberry Pi 3)

#ups=openups2.openups2(1,"LIFEP04")

ahrs=xsens.MTI3()

# if not sensor.init():
        # print "Sensor could not be initialized"
        # exit(1)

# if not sensor.read():
    # print "Sensor read failed!"
    # exit(1)

# print("Pressure: %.2f atm  %.2f Torr  %.2f psi") % (
# sensor.pressure(ms5837.UNITS_atm),
# sensor.pressure(ms5837.UNITS_Torr),
# sensor.pressure(ms5837.UNITS_psi))

# print("Temperature: %.2f C  %.2f F  %.2f K") % (
# sensor.temperature(ms5837.UNITS_Centigrade),
# sensor.temperature(ms5837.UNITS_Farenheit),
# sensor.temperature(ms5837.UNITS_Kelvin))

# freshwaterDepth = sensor.depth() # default is freshwater
# sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
# saltwaterDepth = sensor.depth() # No nead to read() again
# sensor.setFluidDensity(1000) # kg/m^3
# print("Depth: %.3f m (freshwater)  %.3f m (saltwater)") % (freshwaterDepth, saltwaterDepth)

# print("MSL Relative Altitude: %.2f m") % sensor.altitude() # relative to Mean Sea Level pressure in air

# sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)


while 1:
	ahrs.read_data()
	print "roll",ahrs.getEul()[0],"pitch",ahrs.getEul()[1],"yaw",ahrs.getEul()[2]
	print "accx",ahrs.getAcc()[0],"accy",ahrs.getAcc()[1],"accz",ahrs.getAcc()[2]
	acc=np.matrix([[ahrs.getAcc()[0]],[ahrs.getAcc()[1]],[ahrs.getAcc()[2]]])
	g=np.matrix([[0],[0],[9.814]])
	facc=acc-Rxy(-math.radians(ahrs.getEul()[0]),-math.radians(ahrs.getEul()[1]))*g#DA VERIFICARE
	print "Faccx",facc[0],"Faccy",facc[1],"Faccz",facc[2]
	time.sleep(0.1)
	# if sensor.read():
		# print("D: %0.3f m \tT: %0.2f C") % (
		# sensor.depth(), #dovrebbe essere in metri
		# sensor.temperature()), # Default is degrees C (no arguments)
	#time.sleep(0.05)
	#charge=ups.read_charge()
	#print charge	
