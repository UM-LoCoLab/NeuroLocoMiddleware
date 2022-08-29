""" A demo that reads the data of a torque sensor in a loop until ctrl-C """

from GrayDemoModularExos import * # reausable script header

def measure_torque(adc):
    adc.update() # refresh the data
    gdl = GracefulDeathLoop()
    class Looper(object):

        def __init__(self):
            self.t0, self.index = time.time(), 0

        def loop(self):
            adc.update() # refresh the data
            t = time.time()-self.t0
            self.index+=1
            if self.index%100==0:
                print ("τ: %.2f Nm"%(adc.get_torque()))

            if t>10:
                return 0

            return 1 # anything other than 0 is keep looping


    looper = Looper()

    gdl.non_blocking_loop(looper.loop, dt=0.00333333333) #
    # gdl.blocking_loop(looper.loop, dt=0.01) # simple mode for debugging

if __name__ == '__main__':
    with Big100NmFutek(csv_file_name="csv_logs/dither_volts.csv") as adc:
        measure_torque(adc)
