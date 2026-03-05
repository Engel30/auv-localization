import ctypes
libmean = ctypes.CDLL('./libutilities_u.so')
a="\0" * 11
b='\1\0\1'
createRequestMessage=libmean.createRequestMessage
createRequestMessage.argtypes=[ctypes.c_char_p,ctypes.c_byte,ctypes.c_byte,ctypes.c_char_p,ctypes.c_byte]
print libmean.createRequestMessage(a,0,1,b,3)
print ":".join("{:02x}".format(ord(c)) for c in a)
