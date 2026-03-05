import numpy as np

def Rx (roll):
	c=np.cos(roll)
	s=np.sin(roll)
	R=np.matrix([[1, 0, 0],[0, c, -s],[0, s, c]])
	return R 


def Ry (pitch):
	c=np.cos(pitch)
	s=np.sin(pitch)
	R=np.matrix([[c, 0, s],[0, 1, 0],[-s, 0, c]])
	return R 
	
def Rz (yaw):
	c=np.cos(yaw)
	s=np.sin(yaw)
	R=np.matrix([[c, -s, 0],[s, c, 0],[0, 0, 1]])
	return R 
	
def Rxyz (roll,pitch,yaw):
	return Rx(roll)*Ry(pitch)*Rz(yaw)


def Rzyx (roll,pitch,yaw):
	return Rz(yaw)*Ry(pitch)*Rx(roll)

	
def Rxy (roll,pitch):
	return Rx(roll)*Ry(pitch)

def Ryx (roll,pitch):
	return Ry(pitch)*Rx(roll)
