from GrayDemoCommon import *


# Offers conversions to output torque and position
class Big100NmFutek(ADC):
    def __init__(self, csv_file_name=None):
        super().__init__(csv_file_name=csv_file_name)

    def get_torque(self):
        return (self.volts-2.5)*40.

class Small50to1ActPack(ActPacMan):
    def __init__(self, devttyACMport, baudRate, csv_file_name=None,
        printingRate = 10, updateFreq = 1000, shouldLog = False, shouldAuto = 1):
        super().__init__(devttyACMport, baudRate, csv_file_name=csv_file_name,
            printingRate = printingRate, updateFreq = updateFreq,
            shouldLog = shouldLog, shouldAuto = shouldAuto)
        self.K_tau = 0.14 # Nm/A [LeePanRouse2019IROS]
        self.gear_ratio = 50
        self.R_phase = 3./2.*186e-3 # Ohms [LeePanRouse2019IROS] 


    def get_nominal_torque(self):
        return self.get_current_qaxis()*self.K_tau*self.gear_ratio

    def set_nominal_torque(self, torque):
        self.des_current = torque/(self.K_tau*self.gear_ratio)
        des_voltage = self.R_phase * self.des_current + self.get_phi_dot() * self.K_tau
        self.set_voltage_qaxis(des_voltage)

    def get_theta(self):
        return self.get_phi()/self.gear_ratio

class Big9to1ActPack(ActPacMan):
    def __init__(self, devttyACMport, baudRate, csv_file_name=None,
        printingRate = 10, updateFreq = 1000, shouldLog = False, shouldAuto = 1):
        super().__init__(devttyACMport, baudRate, csv_file_name=csv_file_name,
            printingRate = printingRate, updateFreq = updateFreq,
            shouldLog = shouldLog, shouldAuto = shouldAuto)
        self.K_tau = 0.14 # Nm/A [LeePanRouse2019IROS]
        self.R_phase = 3./2.*186e-3 # Ohms [LeePanRouse2019IROS] 
        self.gear_ratio = 9


    def get_nominal_torque(self):
        return self.get_current_qaxis()*self.K_tau*self.gear_ratio

    def set_nominal_torque(self, torque):
        self.des_current = torque/(self.K_tau*self.gear_ratio)
        des_voltage = self.R_phase * self.des_current + self.get_phi_dot() * self.K_tau
        self.set_voltage_qaxis(des_voltage)

    def set_nominal_torque_i(self, torque):
        self.des_current = torque/(self.K_tau*self.gear_ratio)
        
        self.set_current_qaxis(self.des_current)

    def get_phi_dot(self): # Override!
        return self.get_motor_velocity()*np.pi/180.

    def get_theta(self):
        return self.get_phi()/self.gear_ratio

    def get_current_qaxis(self):
        return self.get_motor_current() * 1e-3 * .537/np.sqrt(2.)

    def get_voltage_qaxis(self):
        return self.get_motor_voltage() * 1e-3 * np.sqrt(3./2.)

    def set_voltage_qaxis(self, voltage_qaxis):
        self.set_voltage(voltage_qaxis/(1e-3 * np.sqrt(3./2.)))