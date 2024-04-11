import socket
from time import time
from SoftRealtimeLoop import SoftRealtimeLoop
from threading import Thread
import numpy as np
import struct

BERTEC_MAX_VEL = 3000 # Treadmill max velocity is 3 m/s

class TrapezoidalIntegrator:
    """
    Basic class for a trapezoidal integrator suitable for use in real-time. 
    It internally keeps track of time between samples. Just call int.update(new_value) to tick it forward. 
    I've tested it for this application, but it might break if you use it in some other way. Good luck :). 
    Kevin Best 11/23
    """
    def __init__(self, start_value):
        self._t0 = time()
        self._val_0 = start_value
        self.value = 0.0

    def update(self, new_value):
        """Calls one more iteration of trapezoidal integration. Keeps track of time internally."""
        now = time()
        dt = now - self._t0
        self.value += dt/2*(self._val_0 + new_value)
        
        # Update previous values for next scan
        self._t0 = now
        self._val_0 = new_value

        return self.value
    
    def reset(self):
        self.value = 0.0


class Bertec:
    """
    A class for reading Bertec speed and incline over the network 
    On the Bertec GUI, under Settings, make sure "Remote Control" is enabled
    Katharine Walters 08/23

    Modified 11/1/2023 to also track distance and elevation. - Kevin Best

    WARNING: Only one Pi can connect at a time - Katharine Walters, 03/29/2024
    """
    def __init__(self, viconPC_IP = '141.212.77.30', viconPC_BertecPort = 4000):
        self.destinationIP = viconPC_IP
        self.destinationPort = viconPC_BertecPort

        print("Attempting to connect to socket")

        # Setup TCP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.sock.connect((self.destinationIP, self.destinationPort))

        print("Successfully connected to socket")

        self.thread = Thread(target=self._update, args = ())
        self.stopped = False

        self.belt_speed = [0,0]
        self.incline = 0.0

        # Instantiate trapezoidal integrators to keep track of 
        self._distance_integrator = TrapezoidalIntegrator(self._calculate_absolute_velocity())
        self._elevation_integrator = TrapezoidalIntegrator(self._calculate_vertical_velocity())

        print("Bertec connection initialized")

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
            belt_speedR = int.from_bytes(data[1:3],'big', signed=True)/1000
            belt_speedL = int.from_bytes(data[3:5],'big', signed=True)/1000
            self.belt_speed = [belt_speedL, belt_speedR]
            self.incline = int.from_bytes(data[9:11],'big',signed=True)/100.0
            self._update_odometer()

    def _update_odometer(self):
        """This method uses trapezoidal integration to track distance covered and elevation gained."""
        self._elevation_integrator.update(self._calculate_vertical_velocity())
        self._distance_integrator.update(self._calculate_absolute_velocity())

    def _calculate_vertical_velocity(self):
        avg_belt_speed = np.mean(self.belt_speed)
        return avg_belt_speed*np.sin(np.deg2rad(self.incline))
    
    def _calculate_absolute_velocity(self):
        return np.abs(np.mean(self.belt_speed))

    @property
    def distance(self):
        """Distance traveled since last reset in km."""
        return self._distance_integrator.value/1000.0
    
    @property
    def elevation(self):
        """Elevation gained since last reset in m."""
        return self._elevation_integrator.value
    
    @property
    def speed(self):
        return self._calculate_absolute_velocity()

    def get_treadmill_incline(self):
        # Read treadmill incline 
        return self.incline

    def get_belt_speed(self):
        # Read belt speed 
        return self.belt_speed
    
    def reset_odometer(self):
        """Sets the internal integrals for distance and elevation back to 0."""
        self._distance_integrator.reset()
        self._elevation_integrator.reset()

    def write_command(self, speedR, speedL, incline = None, accR = 0.2, accL = 0.2, maxVel = BERTEC_MAX_VEL, minVel = -BERTEC_MAX_VEL):
        """
        Write speed to treadmill. Code adoptted from MATLAB Bertec GUI 
        at https://github.com/UM-LoCoLab/SelfPacedTMVicon

        Input speedL, speedR, accR, accL, and incline in m/s, m/s^2, and deg
        """

        if incline == None:
            incline = self.incline

        incline = incline * 100
        speedL = speedL*1000        # Speed in mm/s
        speedR = speedR*1000

        # Speed range check
        speedL = min(maxVel, max(minVel, speedL))
        speedR = min(maxVel, max(minVel, speedR))

        accR = accR*1000   # Acceleration in mm/s^2
        accL = accL*1000

        # Formating packet payload 
        format = 0
        speedLL = 0
        speedRR = 0
        accRR = 0
        accLL = 0

        aux = int16toBytes([speedR, speedL, speedRR, speedLL, accR, accL, accRR, accLL, incline])
        secCheck = 255*np.ones((len(aux), ), dtype=int) - aux
        padding = np.zeros((27, ), dtype=int)
        fullPayload = [format, *aux, *secCheck, *padding]

        # Send packet
        packed_data = struct.pack('B' * len(fullPayload), *fullPayload)
        self.sock.sendall(packed_data)

def int16toBytes(intVec):
    """
    A function that converts a vector of int16 (N*1) to a vector of bytes of twice the length (2N*1). 
    Jiefu Zhang 11/23
    """
    byteVec = []
    for int16Data in intVec:
        data = round(int16Data)
        dataInByte = data.to_bytes(2, byteorder='big')
        aux = [dataInByte[0], dataInByte[1]]    
        byteVec.extend(aux)
    return byteVec


if __name__ == '__main__':
    bertec = Bertec()
    bertec.start()

    i = 0
    while i < 200:
        i = i + 1
        speedL, speedR = bertec.get_belt_speed()
        incline = bertec.get_treadmill_incline()
        bertec._write_command(0.5, 0.5)  
        if i >=10:
            i = 0
            print(" speedL ", speedL, 
                  " speedR ", speedR, " incline ", incline, 
                  " distance ", bertec.distance, " elevation ", bertec.elevation,
                  end='\r') 
            
    time.sleep(0.1)
            
    bertec.stop()