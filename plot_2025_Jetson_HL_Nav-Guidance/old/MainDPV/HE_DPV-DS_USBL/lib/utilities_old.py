
import ctypes

libmean = ctypes.CDLL('./lib/libutilities_u.so')
class stateMachine (ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int),("message",ctypes.c_char * 263),("pointer", ctypes.c_int),("counter", ctypes.c_int),
        ("dataLength", ctypes.c_char),("totalLength", ctypes.c_char),("type", ctypes.c_char),("address", ctypes.c_char),
        ("vendor", ctypes.c_char),("model", ctypes.c_char),("command", ctypes.c_char),
    ]
    
class unit (ctypes.Structure):
	_fields_ = [
		("type", ctypes.c_char),("address", ctypes.c_char),("vendor", ctypes.c_char),("model", ctypes.c_char),
		]
	
executeStateMachine=libmean.executeStateMachine
executeStateMachine.argtypes=[ctypes.c_char,ctypes.POINTER(stateMachine),ctypes.c_char]

executeCheckMessage=libmean.executeCheckMessage
executeCheckMessage.argtypes=[ctypes.POINTER(stateMachine),ctypes.POINTER(unit),ctypes.c_char_p,ctypes.c_int,ctypes.c_char_p,ctypes.c_int]

createMessage=libmean.createMessage
createMessage.argtypes=[ctypes.c_char_p,ctypes.POINTER(unit),ctypes.POINTER(stateMachine),ctypes.c_char_p,ctypes.c_byte,ctypes.c_byte]

createRequestMessage=libmean.createRequestMessage
createRequestMessage.argtypes=[ctypes.c_char_p,ctypes.c_byte,ctypes.c_byte,ctypes.c_char_p,ctypes.c_byte]
