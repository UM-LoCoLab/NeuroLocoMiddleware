import socket
from random import randint
import time
from SoftRealtimeLoop import SoftRealtimeLoop
from threading import Thread
import struct
import numpy as np

class Bertec:
    """
    A class for reading Bertec speed and incline over the network 
    On the Bertec GUI, under Settings, make sure "Remote Control" is enabled
    Katharine Walters 08/23
    """
    def __init__(self, viconPC_IP = '141.212.77.30', viconPC_BertecPort = 4000):
        self.destinationIP = viconPC_IP
        self.destinationPort = viconPC_BertecPort

        # Setup TCP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.sock.connect((self.destinationIP, self.destinationPort))

        self.thread = Thread(target=self._update, args = ())
        self.stopped = False

        self.belt_speed = [0,0]
        self.incline = 0.0

    def start(self):
        self.thread.start()
        return self
    
    def stop(self):
        self.stopped = True

    def __del__(self):
        self.thread.join()
        self.sock.close()
        print("Bertec closed")

    def _update(self):
        while True:
            if self.stopped:
                break
            packet = self.sock.recv(32)
            data = list(packet)
            belt_speedR = ((data[1]<<8) + data[2])/1000
            belt_speedL = ((data[3]<<8) + data[4])/1000
            if (belt_speedR > 5) | (belt_speedL > 5): # Correct for negative belt speed
                belt_speedR = (((data[1] - 256)<<8) + data[2])/1000
                belt_speedL = (((data[3] - 256)<<8) + data[4])/1000
            self.belt_speed = [belt_speedL, belt_speedR]
            self.incline = ((data[9]<<8) + data[10])/100
            
    def get_treadmill_incline(self):
        # Read treadmill incline 
        return self.incline

    def get_belt_speed(self):
        # Read belt speed 
        return self.belt_speed

if __name__ == '__main__':
    bertec = Bertec()
    bertec.start()

    i = 0
    loop = SoftRealtimeLoop(dt = 1/100, report=True, fade=0.01)
    t0 = time.time() 
    for t in loop: 
        i = i + 1
        t_curr = time.time() - t0

        speedL, speedR = bertec.get_belt_speed()
        incline = bertec.get_treadmill_incline()

        if i >=10:
            i = 0
            print("time ", t_curr, " speedL ", speedL, " speedR ", speedR, " incline ", incline)
    
    bertec.stop()