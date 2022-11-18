"""
Objects for passing data between two real-time loops.
"""

# import zmq
import socket
import numpy as np
from StatProfiler import StatProfiler

class UdpBase:
    def __init__(self, recv_IP, recv_port, send_IP, send_port, buff_size=1024):
        self.send_sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)
        self.recv_sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)
        self.recv_sock.bind((recv_IP, recv_port))
        self.send_addr = (send_IP, send_port)
        self.buff_size = buff_size
        self.prof = StatProfiler("UDP %s:%d => %s:%d"%(recv_IP, recv_port, send_IP, send_port))

    def send(self, msg):
        self.send_sock.sendto(msg, self.send_addr)

    def recv(self):
        return self.recv_sock.recv(self.buff_size)

class UdpBinarySynchB(UdpBase):
    def __init__(self, recv_IP, recv_port, send_IP, send_port, **kwargs):
        super().__init__(recv_IP, recv_port, send_IP, send_port, **kwargs)
        self.my_count = 0
        self.data_out = None

    def update(self, data_in):
        """ read all messages, then send data."""
        while True:
            try: 
                message = self.recv()
            
                if int(message[1:4])==self.my_count:
                    self.prof.toc() # count received
                    self.my_count+=1
                    self.prof.tic() # sending new count
                    if self.my_count>=1000:
                        self.my_count=0
                self.data_out = np.frombuffer(message[4:])
            except BlockingIOError:
                break
    
        self.send(b"B%03d%s"%(self.my_count, data_in.tobytes()))
        return self.data_out

class UdpBinarySynchA(UdpBase):
    def __init__(self, recv_IP, recv_port, send_IP, send_port, **kwargs):
        super().__init__(recv_IP, recv_port, send_IP, send_port, **kwargs)
        self.my_count = -42
        self.data_out = None

    def update(self, data_in):
        """ read all messages, then send data. """
        while True:
            try: 
                message = self.recv()
                self.my_count=int(message[1:4])
                self.data_out = np.frombuffer(message[4:])
            except BlockingIOError:
                break
        self.send(b"A%03d%s"%(self.my_count, data_in.tobytes()))

        return self.data_out
    