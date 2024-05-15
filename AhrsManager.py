from SoftRealtimeLoop import SoftRealtimeLoop
import time
import csv
import sys, time
import numpy as np
from os.path import realpath
sys.path.append(r'/usr/share/python3-mscl/')    # Path of the MSCL)
import traceback
import mscl
from StatProfiler import SSProfile


class AhrsManager():
    def __init__(self, csv_file_name=None, dt=0.01, port="/dev/ttyACM0", baud = 921600, connection_mode = "USB"):
        self.port = realpath(port) # dereference symlinks
        self.save_csv = not (csv_file_name is None)
        self.csv_file_name = csv_file_name
        self.csv_file = None
        self.csv_writer = None
        self.prevTime = 0.0
        self.R = np.eye(3)
        self.init_R = None
        self.R_prime = None
        self.dt = dt
        self.xd = np.zeros((3,1))
        self.x = np.zeros((3,1))
        self.xd_forget = 0.0
        self.x_forget = 0.0
        self.acc_bias = np.zeros((3,1))
        self.lp_xdd = 0.0
        self.baud = baud
        self.connection_mode = connection_mode  # Allowable arguments are "USB" or "RS232"

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


        self.connection = mscl.Connection.Serial(self.port, self.baud)
        self.node = mscl.InertialNode(self.connection)
        if self.connection_mode == "USB":
            self.node.setToIdle()

        time.sleep(0.05)

        # self.deltaTime = 0
        # self.sampleRate = mscl.SampleRate(1,500)
        #Resume node for streaming
        # self.node.resume()
        #if the node supports AHRS/IMU
        if self.connection_mode == "USB":
            if self.node.features().supportsCategory(mscl.MipTypes.CLASS_AHRS_IMU):
                self.node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU, True)

            #if the self.node supports Estimation Filter
            if self.node.features().supportsCategory(mscl.MipTypes.CLASS_ESTFILTER):
                self.node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER)

            #if the self.node supports GNSS
            if self.node.features().supportsCategory(mscl.MipTypes.CLASS_GNSS):
                self.node.enableDataStream(mscl.MipTypes.CLASS_GNSS)

        # Clean the internal circular buffer. Select timeout to be 500ms
        # self.packets = self.node.getDataPackets(0)
        packets = self.node.getDataPackets(0)

        return self


    def __exit__(self, etype, value, tb):
        """ Closes the file properly """
        if self.save_csv:
            self.csv_file.__exit__(etype, value, tb)
        if self.connection_mode == "USB":
            self.node.setToIdle()
        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    def get_sagittal_angle(self):
        # this is the X-Y plane angle on the device, relative to initial position.

        return 180/np.pi*np.arctan2(self.R_prime[1,0],self.R_prime[0,0])
        return "%.2f degrees"%(180/np.pi*np.arctan2(self.R_prime[1,0],self.R_prime[0,0]))
        return "\n%.2f, %.2f, %.2f\n%.2f, %.2f, %.2f\n%.2f, %.2f, %.2f\n\n"%(
            self.R_prime[0,0], self.R_prime[0,1], self.R_prime[0,2],
            self.R_prime[1,0], self.R_prime[1,1], self.R_prime[1,2],
            self.R_prime[2,0], self.R_prime[2,1], self.R_prime[2,2],
            )

    def update(self):
        t0=time.monotonic()

        microstrainData = self.readIMUnode(timeout=0)# 0ms
        # print([microstrainDatum.keys() for microstrainDatum in microstrainData ])
        for datum in microstrainData:
            if 'orientMatrix' in datum.keys():
                self.R = datum['orientMatrix']
                if self.init_R is None:
                    self.init_R = np.array(self.R)
                self.R_prime = self.R@self.init_R.T
            if 'deltaVelX' in datum.keys():
                self.xdd = self.R_prime.T@np.array([[datum['deltaVelX'], datum['deltaVelY'], datum['deltaVelZ']]]).T*9.81/self.dt
                self.lp_xdd += 0.4*(self.xdd-self.lp_xdd)
                self.xd += (self.xdd - self.xd_forget * self.xd + self.acc_bias) * self.dt
                if np.linalg.norm(self.xdd - self.lp_xdd)<1e-1:
                    self.xd*=0
                    self.acc_bias = -self.lp_xdd

                self.x += (self.xd - self.x_forget * self.x)* self.dt
        # self.R = self.readIMUnode()['orientMatrix']
        # self.R= np.eye(3)
        dur = time.monotonic()-t0
        if self.save_csv:
            self.csv_writer.writerow([time.monotonic()
                , self.R[0,0], self.R[0,1], self.R[0,2]
                , self.R[1,0], self.R[1,1], self.R[1,2]
                , self.R[2,0], self.R[2,1], self.R[2,2]
                ])
        #print(self.R[0,0], self.R[1,1], self.R[2,2])
        return 1

    def start_cal(self):
        self.t0=time.monotonic()
        self.xd = np.zeros((3,1))
        self.x = np.zeros((3,1))
        print('start cal')

    def stop_cal(self):
        cal_time = time.monotonic()-self.t0
        self.acc_bias = -self.xd/cal_time
        self.xd = np.zeros((3,1))
        self.x = np.zeros((3,1))
        self.xd_forget = .01
        self.x_forget = .01
        print('stop cal', self.acc_bias.T)

    def readIMUnode(self, timeout = 0, maxPackets = 0, last_packet_only = False):
        # SSProfile("Get IMU Data Packets").tic()
        packets = self.node.getDataPackets(timeout, maxPackets)
        # SSProfile("Get IMU Data Packets").toc()

        # SSProfile("Read Data Packets").tic() 
        microstrainData = [] 
        if len(packets) > 0 and last_packet_only == True:
            packet = packets[-1]

            microstrainDatum = dict()
            # SSProfile("Loop Through Datapoints").tic()
            for dataPoint in packet.data():
                # print(dataPoint.channelName())
                
                if dataPoint.storedAs() == 0:
                    # SSProfile("Datapoint 0").tic()
                    microstrainDatum[dataPoint.channelName()] = dataPoint.as_float()
                    # SSProfile("Datapoint 0").toc()
                
                elif dataPoint.storedAs() == 3:
                    # SSProfile("Datapoint 3").tic()
                    # print(dir(dataPoint))
                    # ts = dataPoint.as_Timestamp()
                    microstrainDatum[dataPoint.channelName()] = None
                    # SSProfile("Datapoint 3").toc()

                elif dataPoint.storedAs() == 1:
                    # SSProfile("Datapoint 1").tic()
                    # print(dir(dataPoint))
                    ts = dataPoint.as_double()
                    microstrainDatum[dataPoint.channelName()] = ts
                    # SSProfile("Datapoint 1").toc()
                    
                elif dataPoint.storedAs() == 9:
                    # SSProfile("Datapoint 9").tic()
                    mat = dataPoint.as_Matrix()
                    npmat = np.array([[mat.as_floatAt(i,j) for j in range(3)] for i in range(3)])
                    microstrainDatum[dataPoint.channelName()] = npmat
                    # SSProfile("Datapoint 9").toc()
                else:
                    print("no solution for datapoint stored as", dataPoint.storedAs(), dataPoint.channelName())
                    microstrainDatum[dataPoint.channelName()] = None

                # SSProfile("Loop Through Datapoints").toc()
            microstrainData.append(microstrainDatum)
        
        else:
            for packet in packets:
                microstrainDatum = dict()
                # SSProfile("Loop Through Datapoints").tic()
                for dataPoint in packet.data():
                    # print(dataPoint.channelName())
                    
                    if dataPoint.storedAs() == 0:
                        # SSProfile("Datapoint 0").tic()
                        microstrainDatum[dataPoint.channelName()] = dataPoint.as_float()
                        # SSProfile("Datapoint 0").toc()
                    
                    elif dataPoint.storedAs() == 3:
                        # SSProfile("Datapoint 3").tic()
                        # print(dir(dataPoint))
                        # ts = dataPoint.as_Timestamp()
                        microstrainDatum[dataPoint.channelName()] = None
                        # SSProfile("Datapoint 3").toc()

                    elif dataPoint.storedAs() == 1:
                        # SSProfile("Datapoint 1").tic()
                        # print(dir(dataPoint))
                        ts = dataPoint.as_double()
                        microstrainDatum[dataPoint.channelName()] = ts
                        # SSProfile("Datapoint 1").toc()
                        
                    elif dataPoint.storedAs() == 9:
                        # SSProfile("Datapoint 9").tic()
                        mat = dataPoint.as_Matrix()
                        npmat = np.array([[mat.as_floatAt(i,j) for j in range(3)] for i in range(3)])
                        microstrainDatum[dataPoint.channelName()] = npmat
                        # SSProfile("Datapoint 9").toc()
                    else:
                        print("no solution for datapoint stored as", dataPoint.storedAs(), dataPoint.channelName())
                        microstrainDatum[dataPoint.channelName()] = None

                    # SSProfile("Loop Through Datapoints").toc()
                microstrainData.append(microstrainDatum)
        
        # SSProfile("Read Data Packets").toc()
        return microstrainData


    def getTotalPackets(self):
        return self.node.totalPackets()

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
    with AhrsManager(csv_file_name="test_ahrs.csv", port="/dev/ttyAhrsShank_L") as am:
        cal=False
        for i,t in enumerate(SoftRealtimeLoop(dt=1.0/200, report=True)):
            am.update()
            # if t<.01:
            #     am.start_cal()
            # if t>1.5 and not cal:
            #     cal=True
            #     am.stop_cal()
            if i%20==0: print(am.x)
            # print(am.get_sagittal_angle())

if __name__ == '__main__':
    main()