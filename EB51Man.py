""" An object that inherits from ActPackMan and wraps Dephy ExoBoot """

from ActPackMan import ActPackMan
from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import FlexSEA
from ActPackMan import _ActPackManStates
from ActPackMan import NM_PER_AMP
import numpy as np
import math
import time
import csv

from flexsea import fxEnums as fxe  # pylint: disable=no-name-in-module

EB51_DEFAULT_VARIABLES = [ # struct fields defined in flexsea/dev_spec/ActPackState.py
    "state_time",
    "mot_ang", "mot_vel", "mot_acc",
    "mot_volt", "mot_cur", 
    "batt_volt", "batt_curr", 
    "temperature", 
    "status_mn", "status_ex", "status_re",
    "ank_ang"
]

DORSI_TENSION_TORQUE = 0.05   # Apply small torque in dorsiflexion region to maintain belt tension
MAX_CURRENT_AMPS = 15  
MAX_MOTOR_TORQUE = MAX_CURRENT_AMPS * NM_PER_AMP
INERTIA_G_M2 = 0.12  # Motor rotational moment of inertia in grams per meter squared
DEG_PER_RAD = 180/np.pi
MAX_BATTERY_CURRENT_AMPS = 11 

class EB51Man(ActPackMan):
    def __init__(self, devttyACMport, whichAnkle,  
    slack = 0.08, vars_to_log=EB51_DEFAULT_VARIABLES, **kwargs):

        super(EB51Man, self).__init__(devttyACMport, vars_to_log = vars_to_log, **kwargs)

        if whichAnkle not in ['left', 'right']:
            raise RuntimeError("whichAnkle must be left or right") 
        self.whichAnkle = whichAnkle
        self.slack = slack

        self.prevOutputAngle = np.pi/2
        self.currOutputAngle = np.pi/2
        self.prevOutputVel = 0
        self.currOutputVel = 0
        self.prevTime = time.time()
        self.currTime = time.time()
        
        with open("/home/pi/MBLUE/device_side/parameters/MBLUE_Ankle_params.csv", 'r') as params_file:
            params_reader = csv.reader(params_file)
            params = []
            for row in params_reader:
                params.append(row) 
        
        # Fitting break points 
        self.break1 = float(params[1][0])
        self.break2 = float(params[1][1])
        self.break3 = float(params[1][2])
        self.break4 = float(params[1][3])

        # Fitting parameters where a is deg 2, b is deg 1, c is deg 0
        self.angleL1b = float(params[3][1])  # Positive slope linear fit
        self.angleL1c = float(params[3][2])
        self.angleQa = float(params[4][0])  # Quadratic fit
        self.angleQb = float(params[4][1])
        self.angleQc = float(params[4][2])  
        self.angleL2b = float(params[5][1])  # Negative slope linear fit
        self.angleL2c = float(params[5][2])
        
        # Fitting parameters
        self.gearL1 = float(params[7][1])  # Positive constant gear ratio
        self.gearL2a = float(params[8][0])  # Degree 1
        self.gearL2b = float(params[8][1])  # Degree 0
        self.gearL3 = float(params[9][1])   # Negative constant gear ratio        

        # Ankle angle where gear ratio flips signs
        self.beltInflectionAngle =  (-1)*self.gearL2b/self.gearL2a + self.break2   
        
        self.calibrationOffset = 0
        self.calibrationRealignmentComplete = False

        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.K = 0
        self.B = 0
        self.ff = 0



    def update(self):
        " Updates member variables and calls parent update function "
        super().update()

        self.gear_ratio = self._calculate_gear_ratio()  # Update gear ratio for ankle angle output

        if (self.get_output_angle_radians() != self.currOutputAngle) or ((time.time() - self.currTime ) > 0.016):
            self.prevOutputAngle = self.currOutputAngle
            self.prevOutputVel = self.currOutputVel
            self.prevTime = self.currTime
            self.currTime = time.time()
            self.currOutputAngle = self.get_output_angle_radians()
            self.currOutputVel = (self.currOutputAngle - self.prevOutputAngle)/(self.currTime - self.prevTime)

    # Gain setting and control mode switching

    def set_position_gains(self, kp=200, ki=50, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        super().set_position_gains(kp=kp, ki=ki, kd=kd)

    def set_current_gains(self, kp=40, ki=400, ff=128):
        self.kp = kp
        self.ki = ki
        self.ff = ff
        super().set_current_gains(kp = kp, ki = ki, ff = ff)

    def set_impedance_gains_raw_unit_KB(self, kp=40, ki=400, K=300, B=1600, ff=128):
        self.kp = kp
        self.ki = ki
        self.K = K
        self.B = B
        self.ff = ff
        super().set_impedance_gains_raw_unit_KB(kp=kp, ki=ki, K=K, B=B, ff=ff)

    def set_impedance_gains_real_unit_KB(self, kp=40, ki=400, K=0.08922, B=0.0038070, ff=128):
        self.kp = kp
        self.ki = ki
        self.K = K
        self.B = B
        self.ff = ff
        super().set_impedance_gains_real_unit_KB(kp=kp, ki=ki, K=K, B=B, ff=ff)

    # Private functions

    def _calculate_gear_ratio(self):  
        " Private function. Recalculated and reassigned to class variable with each call of update function "
        " Gear ratio only accurate if no slack in belt "
        ankle = self.get_output_angle_radians()
        tolerance = 0.04  # Model tolerance on high and low ends of the angle range
        if self.whichAnkle == 'right':
            ankle = np.pi-ankle
        if (ankle > self.break1-tolerance and ankle < self.break2):
            return self.gearL1
        if (ankle >= self.break2 and ankle < self.break3):
            return self.gearL2a*(ankle - self.break2) + self.gearL2b
        if (ankle >= self.break3 and ankle < self.break4+tolerance):
            return self.gearL3
        print("Warning: gear ratio not updated")
        return self.gear_ratio

    def realign_calibration(self):  # Needs to be completed 
        " Call function to realign calibration when motor turned on "
        print("Realign calibration")
        self.calibrationOffset = 0
        controller_state = self._state

        winding_voltage = 0.8 
        if self.whichAnkle == 'right':
            winding_voltage = (-1) * winding_voltage 
        self.set_voltage_qaxis_volts(winding_voltage)
        loop = SoftRealtimeLoop(dt = 0.01, report=False, fade=0.01)
        for t in loop:
            self.update()
            # print("voltage ", self.get_voltage_qaxis_volts(), " current ", self.get_current_qaxis_amps()) 
            if abs(self.get_current_qaxis_amps()) > 2:
                # print("Current threshold hit")
                break

        self.set_voltage_qaxis_volts(0.0)
        ankle_angle = self.get_output_angle_radians()
        model_motor_angle = self.get_desired_motor_angle_radians(ankle_angle)
        actual_motor_angle = self.get_motor_angle_radians()
        
        self.calibrationOffset = actual_motor_angle - model_motor_angle
        print("model_motor_angle ", model_motor_angle, " actual_motor_angle ", actual_motor_angle, " calibration offset ", self.calibrationOffset)
        
        time.sleep(0.05)
        if abs(self.get_desired_motor_angle_radians(ankle_angle) - actual_motor_angle) > 0.5:
            print("Warning: calibration realignment failed, trying again")
            time.sleep(2)
            self.realign_calibration()
        
        self.calibrationRealignmentComplete = True 
        self._state = controller_state 
        FlexSEA().set_gains(self.dev_id, self.kp, self.ki, self.kd, self.K, self.B, self.ff)

        time.sleep(0.05)
        print("Belt calibration realingment complete")



    # Motor-side variables

    def get_motor_torque_newton_meters(self):
        return self.get_current_qaxis_amps()*NM_PER_AMP

    def set_motor_torque_newton_meters(self, torque):
        battery_current = abs(self.get_current_qaxis_amps()) * abs(self.get_voltage_qaxis_volts()) / self.get_battery_voltage_volts()
        ratio = battery_current / MAX_BATTERY_CURRENT_AMPS
        if ratio > 1: 
            scale = 1/ratio
        else:
            scale = 1
        desired_current = scale * torque/NM_PER_AMP
        if self.whichAnkle == 'right':
            desired_current = (-1) * desired_current 
        return self.set_current_qaxis_amps(desired_current)   
        
    # Tension condition 

    def get_output_angle_degrees(self):
        " Convert ankle angle to degrees "
        return self.get_output_angle_radians() * DEG_PER_RAD

    def get_output_angle_radians(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.ank_ang * 2*np.pi / pow(2,14)

    def get_output_velocity_radians_per_second(self):  
        return self.currOutputVel

    def get_output_acceleration_radians_per_second_squared(self):  
        return (self.currOutputVel - self.prevOutputVel)/(self.currTime - self.prevTime)

    def get_output_torque_newton_meters(self): 
        return self.get_motor_torque_newton_meters()*self.gear_ratio

    #### 

    def set_output_angle_radians(self, angle, slacked = False):  ### Need to check 
        " Function sends command to set motor angle corresponding to ankle angle "
        if self.calibrationRealignmentComplete == False:
            raise RuntimeError("Calibration must be realigned before using position control.")
        if (slacked == True) & (angle <= self.beltInflectionAngle):
            target_angle = angle + self.slack
        elif (slacked == True) & (angle > self.beltInflectionAngle):
            target_angle = angle - self.slack
        elif slacked == False:
            target_angle = angle 
        motor_angle = self.get_desired_motor_angle_radians(target_angle) 
        self.set_motor_angle_radians(motor_angle) 

    def set_output_velocity_radians_per_second(self, vel):  
        raise NotImplemented()

    def set_output_acceleration_radians_per_second_squared(self, acc):
        raise NotImplemented()
    
    def set_output_torque_newton_meters(self, torque):  # Need to include over-current protection
        " Sends command for positive (plantarflexion) torque " 
        if torque < 0:
            torque = 0
            # print("Warning: only positive (plantarflexion) torques allowed")
        motor_torque = torque/self.gear_ratio

        # Motor inertia compensation
        motor_torque = motor_torque + INERTIA_G_M2 * 0.001 * self.get_motor_acceleration_radians_per_second_squared()
        
        if self.gear_ratio < 0:
            motor_torque = DORSI_TENSION_TORQUE   
        if motor_torque > MAX_MOTOR_TORQUE:  
            motor_torque = MAX_MOTOR_TORQUE
        self.set_motor_torque_newton_meters(motor_torque)

    # Slack condition

    def get_desired_motor_angle_radians(self, output_angle):  
        " Calculate motor angle based on ankle angle "
        tolerance = 0.04  # Model tolerance on high and low ends of the angle range
        # if self.whichAnkle == 'right':
        #     output_angle = np.pi-output_angle
        if (output_angle > self.break1 - tolerance) & (output_angle < self.break2):
            return (self.angleL1b*(output_angle - self.break1) + self.angleL1c) + self.calibrationOffset
        if (output_angle >= self.break2) & (output_angle < self.break3):
            return (self.angleQa*pow(output_angle - self.break2, 2) + self.angleQb*(output_angle - self.break2) + self.angleQc) + self.calibrationOffset
        if (output_angle >= self.break3) & (output_angle < self.break4 + tolerance):
            return (self.angleL2b*(output_angle - self.break3) + self.angleL2c) + self.calibrationOffset
        raise RuntimeError("Not valid output angle") 

    def get_slack(self):
        current_ankle_angle = self.get_output_angle_radians()
        tensioned_motor_angle = self.get_desired_motor_angle_radians(current_ankle_angle)
        actual_motor_angle = self.get_motor_angle_radians()
        print("tensioned motor angle ", tensioned_motor_angle, " actual motor angle ", actual_motor_angle)
        return abs(tensioned_motor_angle - actual_motor_angle)

    def set_slack(self, slack):
        " Define the amount of slack (in output/ankle radians) to allow "
        self.slack = slack

    # Motor-side variables 
    τm = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters,
        doc="motor_torque_newton_meters")

    # Output-side variables (ankle)

    θ = property(get_output_angle_radians, set_output_angle_radians)
    θd = property(get_output_velocity_radians_per_second, 
        set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    θdd = property(get_output_acceleration_radians_per_second_squared, 
        set_output_acceleration_radians_per_second_squared,
        doc="output_acceleration_radians_per_second_squared")

    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters,
        doc="output_torque_newton_meters")


