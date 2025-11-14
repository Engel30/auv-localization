#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import select
import time


def _to_bytes(s: str) -> bytes:
    """Encode protocol strings to ASCII bytes (replace non-ASCII just in case)."""
    return s.encode("ascii", errors="replace")


def _to_str(b: bytes) -> str:
    """Decode bytes from socket to str."""
    return b.decode("ascii", errors="replace")


def checksum(message):
    """
    XOR of all bytes from index 1..len-2 (excluded endpoints),
    returned as a 2-digit lowercase hex string.
    Accepts str or bytes; returns str.
    """
    if isinstance(message, str):
        b = message.encode("ascii", errors="replace")
    else:
        b = message

    if len(b) < 2:
        val = 0
    else:
        val = 0
        # note: Python slices are end-exclusive; mirrors original 1..len-2
        for ch in b[1:len(b) - 1]:
            val ^= ch

    return format(val, "02x")


class INSTANT_MSG:
    """
    Helper to build an AT*SENDIM command string.
    - If ext=True:  AT*SENDIM,p{PID},{lung},{Dest_ADD},{flag},{data}
    - Else:         AT*SENDIM,{lung},{Dest_ADD},{flag},{data}
    - If lung >= 65: message = "ERROR"
    """
    def __init__(self, PID, Dest_ADD, flag, lung, data, ext):
        self.PID = PID
        self.address = Dest_ADD
        self.flag = flag
        self.data = data
        self.lung = lung

        if self.lung < 65:
            if ext:
                self.message = f"AT*SENDIM,p{PID},{lung},{Dest_ADD},{flag},{data}"
            else:
                self.message = f"AT*SENDIM,{lung},{Dest_ADD},{flag},{data}"
        else:
            self.message = "ERROR"


class Usbl:
    """
    Minimal TCP/“telnet-like” client for an AT-style device.

    Notes:
    - Lines are sent with '\n' (LF), matching your original code.
    - read_data() blocks until the socket is readable (no timeout).
    """

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_telnet = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_telnet.settimeout(2.0)
        self.last_message = ""

        try:
            self.client_telnet.connect((self.host, self.port))
            print("Connected")
        except Exception as e:
            print("Unable to connect:", e)

        time.sleep(0.5)
        self.write_data("+++ATC")

    def _reconnect(self):
        try:
            self.client_telnet.close()
        except Exception:
            pass
        self.client_telnet = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_telnet.settimeout(2.0)
        try:
            self.client_telnet.connect((self.host, self.port))
            print("Reconnected")
            return True
        except Exception as e:
            print("Unable to connect:", e)
            return False

    def read_data(self):
        """Block until data arrives; return str data or 'ERROR' on failure."""
        socket_list = [self.client_telnet]
        # Wait until readable (no timeout, like original)
        read_sockets, _, _ = select.select(socket_list, [], [])

        for sock in read_sockets:
            if sock is self.client_telnet:
                try:
                    data = sock.recv(4096)
                except Exception as e:
                    print("recv error:", e)
                    if self._reconnect():
                        return "ERROR"
                    return "ERROR"

                if not data:
                    # Connection closed / no data; try to reconnect
                    if self._reconnect():
                        return "ERROR"
                    return "ERROR"
                else:
                    s = _to_str(data)
                    self.last_message = s
                    return s

        return "ERROR"

    def write_data(self, msg):
        """Send one protocol line (adds trailing LF if missing)."""
        if not msg.endswith("\n"):
            msg = msg + "\n"
        try:
            self.client_telnet.sendall(_to_bytes(msg))
        except Exception as e:
            print("send error:", e)
            # Try one reconnect then re-send once
            if self._reconnect():
                try:
                    self.client_telnet.sendall(_to_bytes(msg))
                except Exception as e2:
                    print("send retry failed:", e2)

    def create_data_message(self, data):
        """
        Build '#DS_MES,{data}*{chk}' where chk is the XOR checksum
        computed over '#DS_MES,{data}*' (matching original behavior).
        Returns a str (not sent automatically).
        """
        message = "#DS_MES," + data + "*"
        chk = checksum(message)
        return message + chk

    def parse_msg(self):
        """
        Split last_message by commas, return list.
        (Leaves last_message as the raw line/string from read_data.)
        """
        self.msg_param = self.last_message.split(",")
        return self.msg_param
        # Future parsing:
        # if self.msg_param[0] == "USBLLONG": ...
        # elif self.msg_param[0] == "USBLANGLES": ...
        # elif self.msg_param[0] == "RECVIM": ...
        # elif self.msg_param[0] == "DELIVEREDIM": ...
        # TODO: handle OK/ERROR responses
