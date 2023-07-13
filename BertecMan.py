import socket
from random import randint
import time
from SoftRealtimeLoop import SoftRealtimeLoop

class Bertec:
    """
    A class for reading Bertec speed and incline over the network 
    Katharine Walters 07/23
    """
    def __init__(self, viconPC_IP = '192.168.1.134', viconPC_BertecPort = 4000):

        self.destinationIP = viconPC_IP
        self.destinationPort = viconPC_BertecPort

        # Setup TCP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.sock.bind((self.destinationIP, self.destinationPort))
        self.sock.listen()
        self.conn, self.addr = self.sock.accept()

    def __exit__(self):
        self.sock.close()

    def get_belt_speed(self):
        return self.conn.recv(1024)

if __name__ == '__main__':
    bertec = Bertec()

    loop = SoftRealtimeLoop(dt = 1/200, report=True, fade=0.01)
    t0 = time.time()
    i = 0
    for t in loop: 
        i = i+1

        t_curr = time.time() - t0
        speed = bertec.get_belt_speed()

        if i >= 50:
            i = 0
            print("time ", t_curr)