""" An object that wraps the Dephy ActPack """

import os, sys
import time
import csv
import traceback
import numpy as np
import h5py
import deprecated
from enum import Enum
from math import isfinite
from os.path import realpath

# Dephy library import
from flexsea import fxEnums as fxe  # pylint: disable=no-name-in-module
from flexsea.device import Device

# Version of the ActPackMan library
__version__="1.1.0"

# See ActPackState for all available data
labels = [ # matches varsToStream
    "State time",  
    "Motor angle", "Motor velocity", "Motor acceleration",  
    "Motor voltage", "Motor current",   
    "Battery voltage", "Battery current"
]
DEFAULT_VARIABLES = [ # struct fields defined in flexsea/dev_spec/ActPackState.py
    "state_time",
    "mot_ang", "mot_vel", "mot_acc",
    "mot_volt", "mot_cur", 
    "batt_volt", "batt_curr"
]

MOTOR_CLICKS_PER_REVOLUTION = 16384 
RAD_PER_SEC_PER_GYRO_LSB = np.pi/180/32.8
G_PER_ACCELEROMETER_LSB = 1./8192
NM_PER_AMP = 0.1108 # NM_PER_AMP
RAD_PER_CLICK = 2*np.pi/MOTOR_CLICKS_PER_REVOLUTION
RAD_PER_DEG = np.pi/180.
ticks_to_motor_radians = lambda x: x*(np.pi/180./45.5111)
motor_radians_to_ticks = lambda q: q*(180*45.5111/np.pi)

class _ActPackManStates(Enum):
    VOLTAGE = 1
    CURRENT = 2
    POSITION = 3
    IMPEDANCE = 4

class ActPackMan(object):
    """ (Dephy) Actuator Pack Manager
    Keeps track of a single Dephy Actuator
    """
    
    def __init__(self, devttyACMport, baudRate=230400, csv_file_name=None,
        hdf5_file_name=None, vars_to_log=DEFAULT_VARIABLES, gear_ratio=1.0,
        printingRate = 10, updateFreq = 100, shouldLog = False, logLevel=6, 
        enableThermalTorqueThrottling = True):
        """ Intializes variables, but does not open the stream. """

        #init printer settings
        self.updateFreq = updateFreq
        self.shouldLog = shouldLog
        self.logLevel = logLevel
        self.prevReadTime = time.time()-1/self.updateFreq
        self.dt = 1.0/updateFreq
        self.gear_ratio = gear_ratio

        # self.varsToStream = varsToStream
        self.baudRate = baudRate
        self.named_port = devttyACMport
        self.devttyACMport = realpath(devttyACMport)
        self.csv_file_name = csv_file_name
        self.hdf5_file_name = hdf5_file_name
        self.csv_file = None
        self.csv_writer = None
        self.vars_to_log = vars_to_log
        self.entered = False
        self._state = None
        self.act_pack = None # code for never having updated
        self.device = None

        # Keep track of current control gains internally
        self.Igains_kp = 0
        self.Igains_ki = 0
        self.Igains_ff = 0

        # Instantiate Thermal Model
        self.thermal_model = ThermalMotorModel(temp_limit_windings=90, soft_border_C_windings=10, temp_limit_case=70, soft_border_C_case=10)
        self.torqueThermalScaling = 0
        self.enableThermalTorqueThrottling = enableThermalTorqueThrottling

    ## 'With'-block interface for ensuring a safe shutdown.
    def __enter__(self):
        """ Runs when the object is used in a 'with' block. Initializes the comms."""
        if self.csv_file_name is not None:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"]+self.vars_to_log)
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)

        if self.hdf5_file_name is not None:
            self.hdf5_file = h5py.File(self.hdf5_file_name, 'w')

        self.device = Device(self.devttyACMport, self.baudRate)
        self.device.open(self.updateFreq, log_level=self.logLevel, log_enabled=False)
        print('devID %d streaming from %s (i.e. %s)'%(
            self.device.dev_id, self.devttyACMport, self.named_port))
            
        time.sleep(0.1)
        self._state = _ActPackManStates.VOLTAGE
        self.entered = True
        return self

    def __exit__(self, etype, value, tb):
        """ Runs when leaving scope of the 'with' block. Properly terminates comms and file access."""

        if not (self.device is None):
            print('\nTurning off control for device %s (i.e. %s)'%(self.devttyACMport, self.named_port))
            t0=time.time()
            self.v = 0.0
            self.update()
            time.sleep(1.0/self.updateFreq) # Works
            while(abs(self.i)>0.1):
                self.update()
                self.v = 0.0
                time.sleep(1.0/self.updateFreq)

            del self.device
            self.device = None
            time.sleep(1.0/self.updateFreq)
            print('done.', time.time()-t0)
        
        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)


    ## Critical data reading function. Run update exactly once per loop.

    def update(self):
        # fetches new data from the device
        if not self.entered:
            raise RuntimeError("ActPackMan updated before __enter__ (which begins the streaming)")
        currentTime = time.time()
        self.dt =  currentTime-self.prevReadTime
        if abs(self.dt)<0.25/self.updateFreq:
            print("warning: re-updating twice in less than a quarter of a time-step")
        self.act_pack = self.device.read() # a c-types struct
        self.prevReadTime = currentTime

        # Update thermal model
        self.thermal_model.T_c = self.σ
        self.torqueThermalScaling = self.thermal_model.update_and_get_scale(self.dt, self.i, FOS=1.0)
 
        # Automatically save all the data as a csv file
        if self.csv_file_name is not None:
            self.csv_writer.writerow([time.time()]+[getattr(self.act_pack,x) for x in self.vars_to_log])

        if self.hdf5_file_name is not None:
            raise NotImplemented()

        # Check for thermal fault, bit 2 of the execute status byte per dephy's great logic
        if self.act_pack.status_ex & 0b00000010 == 0b00000010:
            raise RuntimeError("Actpack Thermal Limit Tripped")

    ## Gain Setting and Control Mode Switching (using hidden member self._state)
    """
    The behavior of these gain-setting function is to require setting gains
    before setting the corresponding set-point. Voltage mode requires no
    gains, and therefore can be accessed at any time. Setting a voltage means
    gains need to be re-specified before any other mode can be controlled.
    """

    def set_position_gains(self, kp=200, ki=50, kd=0):
        assert(isfinite(kp) and 0 <= kp and kp <= 1000)
        assert(isfinite(ki) and 0 <= ki and ki <= 1000)
        assert(isfinite(kd) and 0 <= kd and kd <= 1000)
        if self._state != _ActPackManStates.POSITION:
            self.set_voltage_qaxis_volts(0.0)
            self._state=_ActPackManStates.POSITION
        self.device.set_gains(kp, ki, kd, 0, 0, 0)
        self.set_motor_angle_radians(self.get_motor_angle_radians())

    def set_current_gains(self, kp=40, ki=400, ff=128):
        assert(isfinite(kp) and 0 <= kp and kp <= 80)
        assert(isfinite(ki) and 0 <= ki and ki <= 800)
        assert(isfinite(ff) and 0 <= ff and ff <= 128)
        # If this is a new entry into current control, reset values to prevent nonsense
        if self._state != _ActPackManStates.CURRENT:
            self.set_voltage_qaxis_volts(0.0)
            time.sleep(0.1)
            self._state=_ActPackManStates.CURRENT
            self.device.set_gains(kp, ki, 0, 0, 0, ff)
            time.sleep(0.1)
            self.set_current_qaxis_amps(0.0)
        else:
            self.device.set_gains(kp, ki, 0, 0, 0, ff)
        

        # Store the new current gains for later use
        self.Igains_kp = kp
        self.Igains_ki = ki
        self.Igains_ff = ff

    def get_current_gains(self):
        """
        Returns the current gains last set in the actuator as tuple:
        (kp, ki, ff)
        """
        return (self.Igains_kp, self.Igains_ki, self.Igains_ff)

    def set_impedance_gains_raw_unit_KB(self, kp=40, ki=400, K=300, B=1600, ff=128):
        # Use this for integer gains suggested by the dephy website
        assert(isfinite(kp) and 0 <= kp and kp <= 80)
        assert(isfinite(ki) and 0 <= ki and ki <= 800)
        assert(isfinite(ff) and 0 <= ff and ff <= 128)
        assert(isfinite(K) and 0 <= K)
        assert(isfinite(B) and 0 <= B)
        # If this is a new entry into impedance control, reset values to prevent nonsense
        if self._state != _ActPackManStates.IMPEDANCE:
            # Quickly set voltage to 0 as we change modes
            self.set_voltage_qaxis_volts(0.0)
            time.sleep(0.1)

            # Write new gains
            self.device.set_gains(int(kp), int(ki), 0, int(K), int(B), int(ff))
            time.sleep(0.1)
            
            # Update internal mode flag
            self._state=_ActPackManStates.IMPEDANCE

            # Update equilibrium angle to current position.
            # This command also changes the actpack mode to impedance
            self.set_motor_angle_radians(self.get_motor_angle_radians())
        else:
            # If we were already in impedance mode, just change the gains and move on with life
            self.device.set_gains(int(kp), int(ki), 0, int(K), int(B), int(ff))

        # Store the current control gains for later use
        self.Igains_kp = kp
        self.Igains_ki = ki
        self.Igains_ff = ff

    def set_impedance_gains_real_unit_KB(self, kp=40, ki=400, K=0.08922, B=0.0038070, ff=128):
        # This attempts to allow K and B gains to be specified in Nm/rad and Nm s/rad.
        A = 0.00028444
        C = 0.0007812
        Nm_per_rad_to_Kunit = RAD_PER_CLICK/C*1e3/NM_PER_AMP
        Nm_s_per_rad_to_Bunit = RAD_PER_DEG/A*1e3/NM_PER_AMP
        # K_Nm_per_rad = torque_Nm/(RAD_PER_CLICK*delta) = 0.146*1e-3*C*K/RAD_PER_CLICK
        # B_Nm_per_rads = torque_Nm/(vel_deg_sec*RAD_PER_DEG) = 0.146*1e-3*A*B / RAD_PER_DEG

        # Scale commanded impedance parameters based on thermal model
        if self.enableThermalTorqueThrottling:
            thermalScale = self.torqueThermalScaling
        else:
            thermalScale = 1.0
        self.set_impedance_gains_raw_unit_KB(kp=kp, ki=ki, K=K*Nm_per_rad_to_Kunit*thermalScale, B=B*Nm_s_per_rad_to_Bunit*thermalScale, ff=ff)

    def set_output_impedance_gains_real_unit_kb(self, kp=40, ki=400, K=0.08922, B=0.0038070, ff=128):
        """
        Set the impedance gains in joint output units. 
        """
        self.set_impedance_gains_real_unit_KB(kp, ki, K/(self.gear_ratio**2), B/(self.gear_ratio**2), ff)

    def isInImpedanceMode(self):
        """
        Boolean return of if the actuator is in impedance control mode
        """
        return self._state == _ActPackManStates.IMPEDANCE

    def isInPositionmode(self):
        """
        Boolean return of if the actuator is in position control mode
        """
        return self._state == _ActPackManStates.POSITION

    ## Primary getters and setters

    # electrical variables

    def get_battery_voltage_volts(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.batt_volt * 1e-3

    def get_battery_current_amps(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.batt_curr * 1e-3

    def get_voltage_qaxis_volts(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_volt * 1e-3

    def set_voltage_qaxis_volts(self, voltage_qaxis):
        self._state = _ActPackManStates.VOLTAGE # gains must be reset after reverting to voltage mode.
        self.device.send_motor_command(fxe.FX_VOLTAGE, int(voltage_qaxis*1000))

    def get_current_qaxis_amps(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_cur * 1e-3

    def set_current_qaxis_amps(self, current_q):
        if self._state != _ActPackManStates.CURRENT:
            raise RuntimeError("Motor must be in current mode to accept a current command")
        if self.enableThermalTorqueThrottling:
            thermalScale = self.torqueThermalScaling
        else:
            thermalScale = 1.0
        self.device.send_motor_command(fxe.FX_CURRENT, int(current_q*1000.0 * thermalScale))


    # motor-side variables

    def get_motor_angle_radians(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return int(self.act_pack.mot_ang)*RAD_PER_CLICK

    def get_motor_velocity_radians_per_second(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_vel*RAD_PER_DEG # expects deg/sec

    def get_motor_acceleration_radians_per_second_squared(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_acc # expects rad/s/s

    def get_motor_torque_newton_meters(self):
        return self.get_current_qaxis_amps()*NM_PER_AMP

    def set_motor_angle_radians(self, pos):
        """
        Sets either the motor position setpoint or the motor equilibrium angle setpoint
        depending on if in position or impedance mode. 
        Angle is specified in motor radians
        """
        if self._state == _ActPackManStates.POSITION:
            fxMode = fxe.FX_POSITION
        elif self._state == _ActPackManStates.IMPEDANCE:
            fxMode = fxe.FX_IMPEDANCE
        else:   
            raise RuntimeError(
                "Motor must be in position or impedance mode to accept a position setpoint")
        self.device.send_motor_command(fxMode, int(pos/RAD_PER_CLICK))

    def set_motor_velocity_radians_per_second(self, motor_velocity):
        raise NotImplemented() # potentially a way to specify position, impedance, or voltage commands.

    def set_motor_acceleration_radians_per_second_squared(self, motor_acceleration):
        raise NotImplemented() # potentially a way to specify position, impedance, or current commands.

    def set_motor_torque_newton_meters(self, torque):
        return self.set_current_qaxis_amps(torque/NM_PER_AMP)

    # output variables

    def get_output_angle_radians(self):
        return self.get_motor_angle_radians()/self.gear_ratio

    def get_output_velocity_radians_per_second(self):
        return self.get_motor_velocity_radians_per_second()/self.gear_ratio

    def get_joint_encoder_counts(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.ank_ang

    def get_joint_encoder_vel_counts(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.ank_vel

    def get_output_acceleration_radians_per_second_squared(self):
        return self.get_motor_acceleration_radians_per_second_squared()/self.gear_ratio

    def get_output_torque_newton_meters(self):
        return self.get_motor_torque_newton_meters()*self.gear_ratio

    def set_output_angle_radians(self, angle):
        self.set_motor_angle_radians(angle*self.gear_ratio)

    def set_output_velocity_radians_per_second(self, vel):
        self.set_motor_velocity_radians_per_second(vel*self.gear_ratio)

    def set_output_acceleration_radians_per_second_squared(self, acc):
        self.set_motor_acceleration_radians_per_second_squared(acc*self.gear_ratio)

    def set_output_torque_newton_meters(self, torque):
        self.set_motor_torque_newton_meters(torque/self.gear_ratio)

    # other
    def get_temp_celsius(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.temperature*1.0 # expects Celsius

    def get_winding_temperature(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.thermal_model.T_w

    def get_gyro_vector_radians_per_second(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return np.array([[1.0*self.act_pack.gyro_x, self.act_pack.gyro_y, self.act_pack.gyro_z]]).T*RAD_PER_SEC_PER_GYRO_LSB# 1.0/32.8 * np.pi/180

    def get_accelerometer_vector_gravity(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return np.array([[self.act_pack.acc_x, self.act_pack.acc_y, self.act_pack.acc_z]]).T*G_PER_ACCELEROMETER_LSB

    def readGenvars(self):
        """
        Reads the genvars struct and returns an NP array of the first 6 entries.
        This commonly contains the load cell information on the OSL. 
        """
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        rawGenvars = np.array([self.act_pack.genvar_0, self.act_pack.genvar_1,
                        self.act_pack.genvar_2, self.act_pack.genvar_3, 
                        self.act_pack.genvar_4, self.act_pack.genvar_5]) 
        return rawGenvars

    def get_state_time(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.state_time

    def get_status_Re(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.status_re

    def get_status_Mn(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.status_mn
    
    def get_status_Ex(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.status_ex

    status_re = property(get_status_Re)
    status_mn = property(get_status_Mn)
    status_ex = property(get_status_Ex)
    
    ## Greek letter math symbol property interface. This is the good
    #  interface, for those who like code that resembles math. It works best
    #  to use the UnicodeMath plugin for sublime-text, "Fast Unicode Math
    #  Characters" in VS Code, or the like to allow easy typing of ϕ, θ, and
    #  τ.

    # general variables
    state_time = property(get_state_time)

    # electrical variables
    v = property(get_voltage_qaxis_volts, set_voltage_qaxis_volts, doc="voltage_qaxis_volts")
    i = property(get_current_qaxis_amps, set_current_qaxis_amps, doc="current_qaxis_amps")
    vBatt = property(get_battery_voltage_volts, doc="Battery voltage")
    iBatt = property(get_battery_current_amps, doc="Battery current")

    # motor-side variables
    ϕ = property(get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians")
    ϕd = property (get_motor_velocity_radians_per_second, 
        set_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second")
    ϕdd = property(get_motor_acceleration_radians_per_second_squared, 
        set_motor_acceleration_radians_per_second_squared, 
        doc="motor_acceleration_radians_per_second_squared")
    τm = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters,
        doc="motor_torque_newton_meters")

    # output-side variables
    θ = property(get_output_angle_radians, set_output_angle_radians)
    θd = property(get_output_velocity_radians_per_second, 
        set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    θdd = property(get_output_acceleration_radians_per_second_squared, 
        set_output_acceleration_radians_per_second_squared,
        doc="output_acceleration_radians_per_second_squared")
    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters,
        doc="output_torque_newton_meters")
    jointEncoderCounts = property(get_joint_encoder_counts)
    jointEncoderVelocityCounts = property(get_joint_encoder_vel_counts)

    # other
    α = property(get_accelerometer_vector_gravity, doc="accelerometer vector, g")
    ω = property(get_gyro_vector_radians_per_second, doc="gyro vector, rad/s")
    σ = property(get_temp_celsius, doc='housing temp, celsius')
    T_w = property(get_winding_temperature, doc="motor modeled winidng temp, c")

    ## Weird-unit getters and setters

    def set_motor_angle_clicks(self, pos):
        if self._state not in [_ActPackManStates.POSITION, _ActPackManStates.IMPEDANCE]:
            raise RuntimeError(
                "Motor must be in position or impedance mode to accept a position setpoint")
        self.device.send_motor_command(fxe.FX_POSITION, int(pos))

    def get_motor_angle_clicks(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.mot_ang



#### TESTING OUT THERMAL MODEL FROM MBLUE TEAM #####
class ThermalMotorModel():
    def __init__(self, ambient=21, params = dict(), temp_limit_windings=115, soft_border_C_windings=15, temp_limit_case=80, soft_border_C_case=5):
        # The following parameters result from Jack Schuchmann's test with no fans
        self.C_w =  0.20*81.46202695970649
        self.R_WC =  1.0702867186480716
        self.C_c =  512.249065845453
        self.R_CA =  1.9406620046327363
        self.α = 0.393*1/100 #Pure copper. Taken from thermalmodel3.py
        self.R_T_0 = 65# temp at which resistance was measured
        self.R_ϕ_0 = .376 # emirical, from the computed resistance (q-axis voltage/ q-axis current). Ohms
        
        self.__dict__.update(params)
        self.T_w = ambient
        self.T_c = ambient
        self.T_a = ambient
        self.soft_max_temp_windings = temp_limit_windings-soft_border_C_windings
        self.abs_max_temp_windings = temp_limit_windings
        self.soft_border_windings = soft_border_C_windings


        self.soft_max_temp_case = temp_limit_case-soft_border_C_case
        self.abs_max_temp_case = temp_limit_case
        self.soft_border_case = soft_border_C_case

    def update_only(self, dt, I_q_des):
        ## Dynamics:
        # self.C_w * d self.T_w /dt = I2R + (self.T_c-self.T_w)/self.R_WC
        # self.C_c * d self.T_c /dt = (self.T_w-self.T_c)/self.R_WC + (self.T_w-self.T_a)/self.R_CA

        I2R = I_q_des**2*self.R_ϕ_0*(1+self.α*(self.T_w-self.R_T_0)) # accounts for resistance change due to temp.

        dTw_dt = (I2R + (self.T_c-self.T_w)/self.R_WC)/self.C_w
        dTc_dt = ((self.T_w-self.T_c)/self.R_WC + (self.T_a-self.T_c)/self.R_CA)/self.C_c
        self.T_w += dt*dTw_dt
        self.T_c += dt*dTc_dt


    def update_and_get_scale(self, dt, I_q_des, FOS=3.):
        ## Dynamics:
        # self.C_w * d self.T_w /dt = I2R + (self.T_c-self.T_w)/self.R_WC
        # self.C_c * d self.T_c /dt = (self.T_w-self.T_c)/self.R_WC + (self.T_w-self.T_a)/self.R_CA

        I2R_des = FOS*I_q_des**2*self.R_ϕ_0*(1+self.α*(self.T_w-self.R_T_0)) # accounts for resistance change due to temp.
        scale=1.0
        if self.T_w > self.abs_max_temp_windings:
            scale = 0.0
        elif self.T_w > self.soft_max_temp_windings:
            scale *= (self.abs_max_temp_windings - self.T_w)/(self.abs_max_temp_windings - self.soft_max_temp_windings)


        if self.T_c > self.abs_max_temp_case:
            scale = 0.0
        elif self.T_c > self.soft_max_temp_case:
            scale *= (self.abs_max_temp_case - self.T_w)/(self.abs_max_temp_case - self.soft_max_temp_case)

        I2R = I2R_des*scale

        dTw_dt = (I2R + (self.T_c-self.T_w)/self.R_WC)/self.C_w
        dTc_dt = ((self.T_w-self.T_c)/self.R_WC + (self.T_a-self.T_c)/self.R_CA)/self.C_c
        self.T_w += dt*dTw_dt
        self.T_c += dt*dTc_dt

        if scale<=0.0:
            return 0.0
        if scale>=1.0:
            return 1.0
        return np.sqrt(scale) # this is how much the torque should be scaled
