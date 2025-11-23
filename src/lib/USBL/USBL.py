import socket
import select
import time

def checksum(message):
    checksum = 0
    for i in range(1, len(message) - 1):
        checksum ^= ord(message[i])
    if checksum > 15:
        checksum = hex(checksum)[2:]
    else:
        checksum = '0' + hex(checksum)[2:]
    return checksum	

class INSTANT_MSG:
    def __init__(self, PID, Dest_ADD, flag, lung, data, ext):
        self.PID = PID
        self.address = Dest_ADD
        self.flag = flag
        self.data = data
        self.lung = lung
        if self.lung < 65:
            if ext:
                self.message = "AT*SENDIM,p" + PID + "," + str(lung) + "," + Dest_ADD + "," + flag + "," + data
            else:
                self.message = "AT*SENDIM," + str(lung) + "," + Dest_ADD + "," + flag + "," + data
        else:
            self.message = "ERROR"

class Usbl:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_telnet = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_telnet.settimeout(2)
        try:
            self.client_telnet.connect((host, port))
            print("Connected")
        except Exception as e:
            print(f'Unable to connect: {e}')
        time.sleep(0.5)
        self.write_data("+++ATC")
        self.last_message = ""
            
    def read_data(self):
        socket_list = [self.client_telnet]	
        read_sockets, write_sockets, error_sockets = select.select(socket_list, [], [], 0.1)

        for sock in read_sockets:
            if sock == self.client_telnet:
                data = sock.recv(4096)
                if not data:
                    try:
                        self.client_telnet.connect((self.host, self.port))
                    except Exception as e:
                        print(f'Unable to connect: {e}')
                    return "ERROR"
                else:
                    try:
                        decoded_data = data.decode('utf-8', errors='ignore')
                        self.last_message = decoded_data
                        return decoded_data
                    except:
                        self.last_message = str(data)
                        return str(data)
        return None
    
    def write_data(self, msg):
        if not msg.endswith("\n"):
            msg = msg + "\n"
        try:
            self.client_telnet.send(msg.encode('utf-8'))
        except Exception as e:
            print(f"Error sending data: {e}")
        
    def create_data_message(self, data):
        message = "#DS_MES," + data + "*"
        chk = checksum(message)
        message += chk
        return message
        
    def parse_msg(self):
        self.msg_param = self.last_message.split(",")
        return self.msg_param
    
    def close(self):
        self.client_telnet.close()