""" An object that inherits from ActPackMan and wraps Dephy ExoBoot """

from ActPackMan import ActPackMan
from ActPackMan import FlexSEA
from ActPackMan import DEFAULT_VARIABLES
import numpy as np
import time
import csv

class EB51Man(ActPackMan):
    def __init__(self, devttyACMport, baudRate=230400, csv_file_name=None,
        hdf5_file_name=None, vars_to_log=DEFAULT_VARIABLES, gear_ratio=1.0,
        printingRate = 10, updateFreq = 100, shouldLog = False, logLevel=6, slack = 1.0, 
        whiplashProtect = True):

        super(EB51Man, self).__init__(devttyACMport, baudRate=230400, csv_file_name=None,
        hdf5_file_name=None, vars_to_log=DEFAULT_VARIABLES, gear_ratio=1.0,
        printingRate = 10, updateFreq = 100, shouldLog = False, logLevel=6)

        self.slack = slack
        self.whiplashProtect = whiplashProtect
        

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
        


    def update(self):
        # fetches new data from the device
        if not self.entered:
            raise RuntimeError("ActPackMan updated before __enter__ (which begins the streaming)")
        currentTime = time.time()
        if abs(currentTime-self.prevReadTime)<0.25/self.updateFreq:
            print("warning: re-updating twice in less than a quarter of a time-step")
        self.act_pack = FlexSEA().read_device(self.dev_id) # a c-types struct
        self.gear_ratio = self._calculate_gear_ratio()  # Update gear ratio for ankle angle output
        self.prevReadTime = currentTime

        # Automatically save all the data as a csv file
        if self.csv_file_name is not None:
            self.csv_writer.writerow([time.time()]+[getattr(self.act_pack,x) for x in self.vars_to_log])

        if self.hdf5_file_name is not None:
            raise NotImplemented()

    # Private functions

    def _calculate_gear_ratio(self):  
        " Private function. Recalculated and reassigned to class variable with each call of update function "
        " Gear ratio only accurate if no slack in belt "
        ankle = self.get_output_angle_radians()
        if (ankle > self.break1-0.02 and ankle < self.break2):
            return self.gearL1
        if (ankle >= self.break2 and ankle < self.break3):
            return self.gearL2a*(ankle - self.break2) + self.gearL2b
        if (ankle >= self.break3 and ankle < self.break4+0.02):
            return self.gearL3
        print("Warning: gear ratio not updated")
        return self.gear_ratio

        
    # Tension condition

    def get_output_angle_degrees(self):
        " Convert ankle angle to degrees "
        return self.get_output_angle_radians() * 180/np.pi

    def get_output_angle_radians(self):
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.ank_ang * 2*np.pi / pow(2,14)

    def get_output_velocity_radians_per_second(self):  # Needs to be updated to reflect velocity at ankle not motor
        return self.get_motor_velocity_radians_per_second()/self.gear_ratio

    def get_output_acceleration_radians_per_second_squared(self):  # Needs to be updated to reflect acceleration at ankle not motor
        return self.get_motor_acceleration_radians_per_second_squared()/self.gear_ratio

    def get_output_torque_newton_meters(self): # Needs to be updated to reflect output torque at ankle not at motor
        return self.get_motor_torque_newton_meters()*self.gear_ratio

    #### 

    def set_output_angle_radians(self, angle, slacked = False):  ### Need to check 
        " Function sends command to set motor angle corresponding to ankle angle "
        if slacked == True & angle <= self.beltInflectionAngle:
            target_angle = angle + self.slack
        elif slacked == True & angle > self.beltInflectionAngle:
            target_angle = angle - self.slack
        elif slacked == False:
            target_angle = angle 
        motor_angle = self.get_desired_motor_angle_radians(target_angle) 
        self.set_motor_angle_radians(motor_angle) 

    def set_output_velocity_radians_per_second(self, vel):
        raise NotImplemented()

    def set_output_acceleration_radians_per_second_squared(self, acc):
        raise NotImplemented()
    
    def set_output_torque_newton_meters(self, torque):   ## Needs to be updated
        " Sets commanded torque to zero if torque command not executable " 
        if (self.gear_ratio == 0) or (np.sign(self.gear_ratio) != np.sign(torque)):
            torque = 0
            print("Warning: torque command not executable")
        self.set_motor_torque_newton_meters(torque/self.gear_ratio)
        

    # Slack condition

    def get_desired_motor_angle_radians(self, output_angle):   ## Need to check
        " Calculate motor angle based on ankle angle "
        if (output_angle > self.break1-0.02) & (output_angle < self.break2):
            return (self.angleL1b*(output_angle - self.break1) - self.angleL1c)
        if (output_angle >= self.break2) & (output_angle < self.break3):
            return (self.angleQa*pow(output_angle - self.break2, 2) + self.angleQb*(output_angle - self.break2) + self.angleQc)
        if (output_angle >= self.break3) & (output_angle < self.break4+1):
            return (self.angleL2b*(output_angle - self.break3) + self.angleL2c)
        raise RuntimeError("Not valid output angle") 

    def set_slack(self, slack):
        " Define the amount of slack (in output/ankle radians) to allow "
        self.slack = slack

    # Output-side variables (ankle)

    θ = property(get_output_angle_radians, set_output_angle_radians)
    θd = property(get_output_velocity_radians_per_second, 
        set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    θdd = property(get_output_acceleration_radians_per_second_squared, 
        set_output_acceleration_radians_per_second_squared,
        doc="output_acceleration_radians_per_second_squared")

    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters,
        doc="output_torque_newton_meters")


