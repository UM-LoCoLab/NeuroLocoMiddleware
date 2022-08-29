""" A demo that follows a pair of position task trajectories, until ctrl-C """

from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan # reusable header
import numpy as np

def debug_9to1(dev0):
    """ Main demo function. Runs a trajectory in a loop until ctrl-C. """

    print("one device playground")
    dev0.update() # refresh the data

    dev0.enter_position_control(kp=200, ki=0)
    dev0.enter_voltage_control()

    print("initial position", dev0.get_motor_angle())

    

    class Looper(object):

        def __init__(self):
            self.t0, self.index = time.time(), 0
            self.int_theta = 0.
            self.t = 0.
            self.int_err = 0.
  

        def loop(self):
            preamble = "One device: "
            t = time.time()-self.t0
            dt = t-self.t
            self.t = t
            dev0.update()# refresh the data
            # 2 produced: ϕd: 0.31 rad/s τ: 0.19 Nm, i: 0.15 A, i_des: 1.59 A, v: 2.24 V, v_des: 2.26 V
            # 4 produced: ϕd: 0.67 rad/s τ: 0.21 Nm, i: 0.16 A, i_des: 3.17 A, v: 4.49 V, v_des: 4.52 V
            # V  α  6.27*ϕ_dot [empirical]

            # 4.0 Nm -> -3.0 Nm (1.0 Nm)
            #-4.0 Nm -> 3.7 Nm (.3 Nm)
            #des_torque = 0.0 if t<2 else 10*np.sin(6.28*2*(t-2))
            des_torque = 0.0 if t<2 else 18

            err = dev0.get_nominal_torque()-des_torque
            self.int_err += dt*err
            dev0.set_nominal_torque(des_torque-.5*(err)-5.*self.int_err)
            # dev0.set_voltage_qaxis(0.0)
            # dev0.set_voltage(0)

            # dev0.set_motor_angle(dev0.get_motor_angle()+2000)
            self.index+=1

            self.int_theta += dev0.get_phi_dot()*dt / dev0.gear_ratio
            # t: 6.33 s, θ: 254.67 rad, ∫θ: 5.60, ==> factor of 45.4767 needed
            if self.index%100==0:
                print(dev0.get_theta())
                print(dev0.get_nominal_torque())
                print("t: %.2f s, θ: %.2f rad, ϕd: %.2f rad/s, τ_nom: %.2f Nm, i: %.2f A, i_des: %.2f A, v: %.2f V, v_des: %.2f V"%(
                    t, dev0.get_theta(), dev0.get_phi_dot(),
                    dev0.get_nominal_torque(),
                    dev0.get_current_qaxis(), dev0.des_current,
                    dev0.get_voltage_qaxis(), dev0.get_last_qaxis_voltage_command()
                    ))


            if t>4:
                return 0

            return 1 # anything other than 0 is keep looping


    looper = Looper()
    SoftRealtimeLoop().run(looper.loop, dt=0.00333333333) #
    # SoftRealtimeLoop().blocking_loop(looper.loop, dt=0.01) # simple mode for debugging


if __name__ == '__main__':
    """ This is the initialization for the demo"""
    loadSuccess = fxUtil.loadFlexsea()
    baudRate = 230400
    ports = ["/dev/ttyACM0"]


    with ActPackMan(ports[0], baudRate, gear_ratio=9, printingRate=200, csv_file_name="csv_logs/test9to1/new_actuator001.csv") as dev0:
        debug_9to1(dev0, adc)
    # no need to clean up, it's all handled by the __exit__ method of ActPacMan

