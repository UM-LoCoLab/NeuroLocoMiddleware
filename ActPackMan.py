""" An object that wraps the Dephy ActPack """

import os, sys
import time
import csv
import traceback
import numpy as np
import h5py
import deprecated
from enum import Enum

# Dephy library import
from flexsea import fxUtils as fxu  # pylint: disable=no-name-in-module
from flexsea import fxEnums as fxe  # pylint: disable=no-name-in-module
from flexsea import flexsea as flex

# Version of the ActPackMan library
__version__="1.0.0"



class FlexSEA(flex.FlexSEA):
    """ A singleton class that prevents re-initialization of FlexSEA """
    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            print("making a new one")
            cls._instance = flex.FlexSEA()
        return cls._instance

    def __init__(self):
        pass


# See demo scripts for more streamable data
labels = [ # matches varsToStream
    "State time",  
    "Motor angle", "Motor velocity", "Motor acceleration",  
    "Motor voltage", "Motor current",   
    "Battery voltage", "Battery current"
]
# varsToStream = [ # integer enums defined in pyFlexsea_def.py
#     fxUtil.FX_STATETIME,
#     fxUtil.FX_ENC_ANG, fxUtil.FX_ENC_VEL, fxUtil.FX_ENC_ACC,
#     fxUtil.FX_MOT_VOLT, fxUtil.FX_MOT_CURR, 
#     fxUtil.FX_BATT_VOLT, fxUtil.FX_BATT_CURR
# ]
ticks_to_motor_radians = lambda x: x*(np.pi/180./45.5111)
motor_radians_to_ticks = lambda q: q*(180*45.5111/np.pi)

class _ActPacManStates(Enum):
    VOLTAGE = 1
    CURRENT = 2
    POSITION = 3
    IMPEDANCE = 4


class ActPacMan(object):
    """ (Dephy) Actuator Pack Manager
    Keeps track of a single Dephy Actuator
    """

    # Class variable that keeps track of the number of connections
    CURRENT_PORT_ID = 0
    
    def __init__(self, devttyACMport, baudRate=230400, csv_file_name=None, hdf5_file_name=None, gear_ratio=1.0, printingRate = 10, updateFreq = 100, shouldLog = False, logLevel=6):
        """ Intializes stream and printer """
        #init printer settings
        self.counter = 0
        self.prev_data = None
        self.labels = labels
        self.data = None
        self.rate = printingRate
        self.updateFreq = updateFreq
        self.shouldLog = shouldLog
        self.logLevel = logLevel
        self.prevReadTime = time.time()
        self.gear_ratio = gear_ratio

        # load stream data
        self.devId = None
        # self.varsToStream = varsToStream
        self.port = ActPacMan.CURRENT_PORT_ID
        self.baudRate = baudRate
        self.devttyACMport = devttyACMport
        self.devId = None
        self.save_csv = not (csv_file_name is None)
        self.csv_file_name = csv_file_name
        self.save_hdf5 = not (hdf5_file_name is None)
        self.hdf5_file_name = hdf5_file_name
        self.csv_file = None
        self.csv_writer = None
        self.entered = False
        self.state = None

    def __enter__(self):

        if self.save_csv:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"]+self.labels)
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)

        if self.save_hdf5:
            self.hdf5_file = h5py.File(self.hdf5_file_name, 'w')


        fxs = FlexSEA() # grab library singleton (see impl. in ActPackMan.py)
        # dev_id = fxs.open(port, baud_rate, log_level=6)
        self.dev_id = fxs.open(self.devttyACMport, self.baudRate, log_level=self.logLevel)
        # fxs.start_streaming(dev_id, 100, log_en=False)
        print('devID', self.dev_id)
        # Start stream
        fxs.start_streaming(self.dev_id, self.updateFreq, log_en=self.shouldLog)

        # app_type = fxs.get_app_type(dev_id)
        self.app_type = fxs.get_app_type(self.dev_id)
        print(self.app_type)

        # Gains are, in order: kp, ki, kd, K, B & ff
        self.set_current_gains(self, kp=40, ki=400, ff=128)
        self.entered = True
        return self

    def _set_gains(self, kp=40, ki=400, kd=0, K=0, B=0, ff=128):
        # dev.set_gains(kp=40, ki=400, ff=128)
        FlexSEA().set_gains(self.dev_id, kp, ki, kd, K, B, ff)

    def set_position_gains(self, kp=200, ki=50, kd=0):
        self.state=_ActPacManStates.POSITION
        FlexSEA().set_gains(self.dev_id, kp, ki, kd, 0, 0, 0)

    def set_current_gains(self, kp=40, ki=400, ff=128)
        self.state=_ActPacManStates.CURRENT
        FlexSEA().set_gains(self.dev_id, kp, ki, 0, 0, 0, ff)

    def set_impedance_gains(self, kp=40, ki=400, K=300, B=1600, ff=128):
        A = 0.00028444
        C = 0.0007812
        self.state=_ActPacManStates.CURRENT
        FlexSEA().set_gains(self.dev_id, kp, ki, 0, K, B, ff)


    def update(self, data_labels = None):
        data = None
        currentTime = time.time()
        if abs(currentTime - self.prevReadTime) >= (1/self.updateFreq):
            self.data = fxUtil.fxReadDevice(self.devId,self.varsToStream)
        self.prevReadTime = currentTime
        if data_labels != None:
            data_index =[]
            for label in data_labels:
                data_index.append(self.varsToStream.index(label))
            data = [self.data[index] for index in data_index]   
        else:
            data = self.data

        # Automatically save all the data as a csv file
        if self.save_csv:
            self.csv_writer.writerow([time.time()]+self.data)

        return data

    def get(self, label):
        return self.data[self.varsToStream.index(label)]

    def get_state_time(self):
        return self.get(fxUtil.FX_STATETIME)

    def get_motor_angle(self):
        return self.get(fxUtil.FX_ENC_ANG)

    def set_motor_angle(self, position):
        fxUtil.setPosition(self.devId, position)

    def get_motor_velocity(self):
        return self.get(fxUtil.FX_ENC_VEL)

    def get_motor_acceleration(self):
        return self.get(fxUtil.FX_ENC_ACC)

    def get_motor_voltage(self):
        return self.get(fxUtil.FX_MOT_VOLT)
        
    def get_motor_current(self):
        return self.get(fxUtil.FX_MOT_CURR)

    def get_battery_voltage(self):
        return self.get(fxUtil.FX_BATT_VOLT)

    def get_battery_current(self):
        return self.get(fxUtil.FX_BATT_CURR)

    @deprecated(version="1.0.0", reason="Dephy no longer has explicit modes. Use set_gains to change control gains.")
    def enter_position_control(self, kp=50, ki=3):
        assert (not(self.data is None))
        self.init_position = self.get_motor_angle()
        fxUtil.setPosition(self.devId, self.get_motor_angle())
        fxUtil.setControlMode(self.devId, fxUtil.CTRL_POSITION)
        fxUtil.setPosition(self.devId, self.get_motor_angle())
        fxUtil.setGains(self.devId, kp, ki, 0, 0)

    def zero_position(self):
        # not tare, since there is no counterweight
        self.init_position = self.get_motor_angle()

    def enter_no_control(self):
        fxUtil.setControlMode(self.devId, fxUtil.CTRL_NONE)

    def set_position(self, position):
        fxUtil.setPosition(self.devId, self.init_position+position)

    def set_phi_desired(self, phi):
        self.set_position(motor_radians_to_ticks(phi))

    def get_position(self):
        return self.get_motor_angle()-self.init_position


    def get_current_qaxis(self):
        return self.get_motor_current() * 1e-3 * .537/np.sqrt(2.)


    def set_current_qaxis(self, current_q):
        fxUtil.setMotorCurrent(self.devId, current_q/( 1e-3 * .537/np.sqrt(2.)))



    def get_voltage_qaxis_volts(self):
        return self.get_motor_voltage() * 1e-3 * np.sqrt(3./2.)

    def set_voltage_qaxis_volts(self, voltage_qaxis):
        self.set_voltage(voltage_qaxis/(1e-3 * np.sqrt(3./2.)))

    @deprecated(version="1.0.0", reason="unclear units (V). Use get_voltage_qaxis_volts")
    def get_voltage_qaxis(self):
        return self.get_voltage_qaxis_volts()

    @deprecated(version="1.0.0", reason="unclear units (V). Use set_voltage_qaxis_volts")
    def set_voltage_qaxis(self, voltage_qaxis): # volts
        self.set_voltage_qaxis_volts(voltage_qaxis)

    ## Short math symbol versions
    v = property(get_voltage_qaxis_volts, set_voltage_qaxis_volts, doc="voltage_qaxis_volts")
    ϕ = property(get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians")
    ϕd = property (get_motor_velocity_radians_per_second, 
        set_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second")
    ϕdd = property(get_motor_acceleration_radians_per_second_squared, 
        set_motor_acceleration_radians_per_second_squared, 
        doc="motor_acceleration_radians_per_second_squared")
    θ = property(get_output_angle_radians, set_output_angle_radians)
    θd = property(get_output_velocity_radians_per_second, 
        set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    θdd = property(get_output_acceleration_radians_per_second_squared, 
        set_output_acceleration_radians_per_second_squared,
        doc="output_acceleration_radians_per_second_squared")
    τm = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters,
        doc="motor_torque_newton_meters")
    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters,
        doc="output_torque_newton_meters")

    def get_phi(self):
        return ticks_to_motor_radians(self.get_position())

    def get_phi_dot(self):
        return ticks_to_motor_radians(self.get_motor_velocity())*1000 # only for older motors. 


    @deprecated(version="1.0.0", reason="Unclear units. Try set_voltage_milliamps")
    def set_voltage(self, voltage): # mA

    def set_voltage(self, voltage): # mA
        fxs = FlexSEA() #singleton
        fxs.send_motor_command(self.dev_id, fxe.FX_NONE, voltage)
        fxUtil.setMotorVoltage(self.devId, voltage)
        self.last_sent_voltage = voltage

    def get_last_qaxis_voltage_command(self):
        return self.last_sent_voltage * 1e-3 * np.sqrt(3./2.)

    def printData(self, clear_terminal = True, message = ""):
        """ Prints data with a predetermined delay, data must be updated before calling this function """
        if clear_terminal:
            clearTerminal()
        if message != "":
            print(message)
        if(self.counter% self.rate == 0):
            fxUtil.printData(self.labels,self.data)
            self.prev_data = self.data
        else:
            fxUtil.printData(self.labels,self.prev_data)
        self.counter += 1
    
    def __exit__(self, etype, value, tb):
        """ Closes stream properly """

        if not (self.dev_id is None):
            print('Turning off control for device %s'%self.devttyACMport)
            fxs = FlexSEA() # singleton
            # dev.__exit__()
            # dev.set_voltage(0.0) # voltage in Volts
            # fxs.send_motor_command(dev_id, fxe.FX_NONE, 0)
            # sleep(0.5)
            # fxs.close(dev_id)
            fxs.send_motor_command(self.devId, fxe.FX_NONE, 0) # 0 mV
            self.set_voltage(0.0)
            print('sleeping')
            sleep(0.5) # I want to delete this
            print('done sleeping')
            print('Position control ended for device %s'%self.devttyACMport)
            fxs.close(self.devId)
        
        if self.save_csv:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)



def main():
    with ActPacMan('/dev/ttyACM0', 230400) as dev0:
        pass





def main():
    fxs = FlexSEA()
    port = '/dev/ttyACM0'
    baud_rate = 230400
    hold_current=None
    time=6
    time_step=0.1
    if hold_current is None:
        hold_current = [1000] # mA

    # dev = ActPacMan('/dev/ttyACM0', 230400)
    with ActPacMan('/dev/ttyACM0', 230400) as dev:
        # dev.__enter__()
            # dev_id = fxs.open(port, baud_rate, log_level=6)
            # fxs.start_streaming(dev_id, 100, log_en=False)
            # app_type = fxs.get_app_type(dev_id)

            # print("Setting controller to current...")
            # # Gains are, in order: kp, ki, kd, K, B & ff
            # # dev.set_gains(kp=40, ki=400, ff=128)
            # fxs.set_gains(dev_id, 40, 400, 0, 0, 0, 128)

        sleep(0.5) # can we delete this?

        prev_current = hold_current[0]
        num_time_steps = int(time / time_step)

        for current in hold_current:
            for i in range(num_time_steps):
                des_current = int(
                    (current - prev_current) * (i / float(num_time_steps)) + prev_current
                )

                # dev.set_current(des_current/1000.) # current in Amps
                fxs.send_motor_command(dev_id, fxe.FX_CURRENT, des_current)
                sleep(time_step)
                # dev.update()
                act_pack = fxs.read_device(dev_id)
                fxu.clear_terminal()
                print("Desired (mA):         ", des_current)
                # print("Measured (A):        ", dev.get_current())
                print("Measured (mA):        ", act_pack.mot_cur)
                print("Difference (mA):      ", (act_pack.mot_cur - des_current), "\n")

                # dev.dephy_print_device()
                fxu.print_device(act_pack, app_type)
            prev_current = current

        print("Turning off current control...")
        # Ramp down first
        ramp_down_steps = 50
        for step in range(ramp_down_steps):
            des_current = prev_current * (ramp_down_steps - step) / ramp_down_steps
            # dev.set_current(des_current/1000.) # current in Amps
            fxs.send_motor_command(dev_id, fxe.FX_CURRENT, des_current)
            # dev.update()
            act_pack = fxs.read_device(dev_id)
            fxu.clear_terminal()
            print("Desired (mA):         ", des_current)
            print("Measured (mA):        ", act_pack.mot_cur)
            print("Difference (mA):      ", (act_pack.mot_cur - des_current), "\n")
            # dev.dephy_print_device()
            fxu.print_device(act_pack, app_type)
            sleep(time_step)

        # When we exit we want the motor to be off

    # dev.__exit__()
    # dev.set_voltage(0.0) # voltage in mVolts
    fxs.send_motor_command(dev_id, fxe.FX_NONE, 0)
    sleep(0.5)
    fxs.close(dev_id)

if __name__ == '__main__':
    from time import sleep
    from SoftRealtimeLoop import SoftRealtimeLoop # helps with the shutdown process
    main()