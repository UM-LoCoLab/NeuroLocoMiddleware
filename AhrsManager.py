# Forked at eed6311827 from PB-EKF-exo-controller
from GracefulDeathLoop import GracefulDeathLoop
import time
import csv
import sys, time
import numpy as np
sys.path.append(r'/usr/share/python3-mscl/')    # Path of the MSCL
import traceback
import mscl



class AhrsManager():
    def __init__(self, csv_file_name=None, port="/dev/ttyACM0"):
        self.port = port
        self.save_csv = not (csv_file_name is None)
        self.csv_file_name = csv_file_name
        self.csv_file = None
        self.csv_writer = None
        self.prevTime = 0.0
        self.init_R = None
        self.R_prime = None

    def __enter__(self):
        if self.save_csv:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time",
                    "r00", "r01", "r02",
                    "r10", "r11", "r12",
                    "r20", "r21", "r22"])
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)


        self.connection = mscl.Connection.Serial(self.port, 921600)
        self.node = mscl.InertialNode(self.connection)
        # self.node.setToIdle()


        # Clean the internal circular buffer. Select timeout to be 500ms
        # self.packets = self.node.getDataPackets(500)

        # self.deltaTime = 0
        # self.sampleRate = mscl.SampleRate(1,500)
        #Resume node for streaming
        # self.node.resume()
        #if the node supports AHRS/IMU
        if self.node.features().supportsCategory(mscl.MipTypes.CLASS_AHRS_IMU):
            self.node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)

        #if the self.node supports Estimation Filter
        if self.node.features().supportsCategory(mscl.MipTypes.CLASS_ESTFILTER):
            self.node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER)

        #if the self.node supports GNSS
        if self.node.features().supportsCategory(mscl.MipTypes.CLASS_GNSS):
            self.node.enableDataStream(mscl.MipTypes.CLASS_GNSS)

        return self


    def __exit__(self, etype, value, tb):
        """ Closes the file properly """
        if self.save_csv:
            self.csv_file.__exit__(etype, value, tb)
        self.node.setToIdle()
        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    def get_sagittal_angle(self):
        # R[0,0] 0 in DOWN_DEFAULT, -1 in HIP_FLEXED_90
        # R[1,1] depends on yaw
        # R[2,2] depends on roll

        return 180/np.pi*np.arctan2(self.R_prime[1,0],self.R_prime[0,0])
        return "%.2f degrees"%(180/np.pi*np.arctan2(self.R_prime[1,0],self.R_prime[0,0]))
        return "\n%.2f, %.2f, %.2f\n%.2f, %.2f, %.2f\n%.2f, %.2f, %.2f\n\n"%(
            self.R_prime[0,0], self.R_prime[0,1], self.R_prime[0,2],
            self.R_prime[1,0], self.R_prime[1,1], self.R_prime[1,2],
            self.R_prime[2,0], self.R_prime[2,1], self.R_prime[2,2],
            )

    def update(self):
        t0=time.time()

        microstrainData = self.readIMUnode(timeout=500)
        # print([microstrainDatum.keys() for microstrainDatum in microstrainData ])
        for datum in microstrainData:
            if 'orientMatrix' in datum.keys():
                self.R = datum['orientMatrix']
        if self.init_R is None:
            self.init_R = np.array(self.R)
        self.R_prime = self.R@self.init_R.T
        # self.R = self.readIMUnode()['orientMatrix']
        # self.R= np.eye(3)
        dur = time.time()-t0
        if self.save_csv:
            self.csv_writer.writerow([time.time()
                , self.R[0,0], self.R[0,1], self.R[0,2]
                , self.R[1,0], self.R[1,1], self.R[1,2]
                , self.R[2,0], self.R[2,1], self.R[2,2]
                ])
        #print(self.R[0,0], self.R[1,1], self.R[2,2])
        return 1


    def readIMUnode(self, timeout = 500):
        packets = self.node.getDataPackets(timeout)
        microstrainData = []
        for packet in packets:
            microstrainDatum = dict()
            for dataPoint in packet.data():
                if dataPoint.storedAs() == 0:
                    microstrainDatum[dataPoint.channelName()] = dataPoint.as_float()
                
                elif dataPoint.storedAs() == 3:
                    # print(dir(dataPoint))
                    # ts = dataPoint.as_Timestamp()
                    microstrainDatum[dataPoint.channelName()] = None

                elif dataPoint.storedAs() == 1:
                    # print(dir(dataPoint))
                    ts = dataPoint.as_double()
                    microstrainDatum[dataPoint.channelName()] = ts
                    
                elif dataPoint.storedAs() == 9:
                    mat = dataPoint.as_Matrix()
                    npmat = np.array([[mat.as_floatAt(i,j) for j in range(3)] for i in range(3)])
                    microstrainDatum[dataPoint.channelName()] = npmat
                else:
                    print("no solution for datapoint stored as", dataPoint.storedAs(), dataPoint.channelName())
                    microstrainDatum[dataPoint.channelName()] = None
            microstrainData.append(microstrainDatum)
        return microstrainData

        print(microstrainData.keys())
        if self.prevTime is None:
            self.prevTime= microstrainData['estFilterGpsTimeTow']
        IMU = {
        'roll': (microstrainData['estRoll'], 'rad'),
        'pitch': (microstrainData['estPitch'], 'rad'),
        'yaw': (microstrainData['estYaw'], 'rad'),
        'omega_x': (microstrainData['estAngularRateX'], 'rad/sec'),
        'omega_y': (microstrainData['estAngularRateY'], 'rad/sec'),
        'omega_z': (microstrainData['estAngularRateZ'], 'rad/sec'),
        'acc_x': (microstrainData['estLinearAccelX'], 'm/s/s'),
        'acc_y': (microstrainData['estLinearAccelY'], 'm/s/s'),
        'acc_z': (microstrainData['estLinearAccelZ'], 'm/s/s'),
        't': (microstrainData['estFilterGpsTimeTow'], 'sec'),
        'dt': (microstrainData['estFilterGpsTimeTow']-self.prevTime, 'sec'),
        'grav_x': (microstrainData['estGravityVectorX'], 'm/s/s'),
        'grav_y': (microstrainData['estGravityVectorY'], 'm/s/s'),
        'grav_z': (microstrainData['estGravityVectorZ'], 'm/s/s'),
        'orientMatrix': (microstrainData['estOrientMatrix'], 'mat')
        }
        self.prevTime= microstrainData['estFilterGpsTimeTow']
        return IMU

    def get_data(self):
        init_time = time.perf_counter_ns()
        #get the data in first packet from the node, with a timeout of 500 milliseconds
        data = self.readIMUnode(timeout = 500)           
        delta_time = (time.perf_counter_ns() - init_time)*1e-6

        self.grav_x = data['grav_x'][0]/9.81
        self.grav_y = data['grav_y'][0]/9.81
        self.grav_z = data['grav_z'][0]/9.81

        return data, delta_time

    def get_euler_angles(self):
        self.roll_usedef = -np.arccos(np.dot(  np.array([0,1,0]) , np.array([self.grav_x,self.grav_y,self.grav_z])  )) + np.pi/2
        self.pitch_usedef = np.arctan2(self.grav_y, self.grav_x)
        eulerAngles = (self.roll_usedef,self.pitch_usedef,self.yaw_usedef)
        # eulerAngles = extractEulerAngles(R_update)

        return eulerAngles


def main():
    with AhrsManager(csv_file_name="test_ahrs.csv", port="/dev/ttyACM0") as am:
        GracefulDeathLoop().non_blocking_loop(lambda: (am.update(), print(am.get_sagittal_angle())), dt=1.0/500.) 

if __name__ == '__main__':
    main()