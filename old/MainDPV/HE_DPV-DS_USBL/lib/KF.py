#!/usr/bin/env python

import numpy.matlib
import numpy as np
import math
import time
from RMatrix import *

def A_m(roll,pitch,yaw,T):
	A=np.identity(9)
	A[0:3,3:6]=	T*Rxyz(roll,pitch,yaw)
	A[0:3,6:9]=	T*T*Rxyz(roll,pitch,yaw)/2
	A[3:6,6:9]=	T*np.identity(3)
	return A

def B_m(n,m,T):
	B=np.matlib.zeros((n,m))
	B[0:3,0:3]=	T*T*T/6*np.identity(3)
	B[3:6,0:3]= T*T/2*np.identity(3)
	B[6:9,0:3]=	T*np.identity(3)
	return B
	
class KF(object):
	def __init__(self,Q,R1,R2,C1,C2,A=A_m,B=B_m,T=0.05,n=9,m=3):
		self.n=n
		self.m=m
		self.T=T
		self.Pkk=np.identity(n)
		self.x=np.matlib.zeros((n,1))
		self.A=A
		self.B=B
		self.C1=C1
		self.C2=C2
		self.Q=Q
		self.R1=R1
		self.R2=R2
		self.t=time.time()
		
	def setX(self, X):
		self.x=X
		
	def getX(self):
		return self.x
		
	def update(self,yy,roll,pitch,yaw,queue):
		t_iter=time.time()
		tc=t_iter-self.t
		#print tc
		self.t=t_iter
		A=self.A(roll,pitch,yaw,tc)
		B=self.B(self.n,self.m,tc)
		Pk1k=A*self.Pkk*np.matrix.transpose(A)+B*self.Q*np.matrix.transpose(B)
		Xk1k=A*self.x
		if queue:
			Kk1=Pk1k*np.matrix.transpose(self.C1)*np.linalg.inv(self.C1*Pk1k*np.matrix.transpose(self.C1)+self.R1)
			self.x=Xk1k+Kk1*(yy-self.C1*Xk1k)
			self.Pkk=(np.identity(self.n)-Kk1*self.C1)*Pk1k
		else:
			Kk1=Pk1k*np.matrix.transpose(self.C2)*np.linalg.inv(self.C2*Pk1k*np.matrix.transpose(self.C2)+self.R2)
			self.x=Xk1k+Kk1*(yy-self.C2*Xk1k)
			self.Pkk=(np.identity(self.n)-Kk1*self.C2)*Pk1k			
		#print self.x
		#print self.P		
		#print time.time()-self.t
	
# Q=np.identity(3)*0.1
# R1=np.identity(3)*0.05
# R2=np.identity(6)*0.5
# R2[3:6,3:6]=R2[3:6,3:6]*0.2
# C1=np.matrix('0, 0, 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1')
# C2=np.matrix('0, 0, 0, 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1; 1, 0, 0, 0, 0, 0, 0, 0, 0; 0, 1, 0, 0, 0, 0, 0, 0, 0; 0, 0, 1, 0, 0, 0, 0, 0, 0')
# d=KF(Q,R1,R2,C1,C2)
# #test iterazione solo acc
# for i in range(1,50):
	# y=np.matrix('0.1;0;0')
	# d.update(y,0,0,math.pi/2,True)
	# time.sleep(0.1)
# # #test iterazione acc e pos
# # time.sleep(1)
# # y=np.matrix('0;0;0;2;0;0')
# # d.update(y,0,0,0,False)
# # for i in range(1,50):
	# # y=np.matrix('0.1;0;0;2;0;0')
	# # d.update(y,0,0,0,False)
	# # time.sleep(0.1)
