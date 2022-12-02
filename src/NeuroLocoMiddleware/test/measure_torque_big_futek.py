""" A demo that reads the data of a torque sensor in a loop until ctrl-C """
from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from AdcManager import AdcManager
import csv

class Big100NmFutek(AdcManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def get_torque(self):
        return (self.volts-2.5)*40.

def measure_torque(adc):
    adc.update()
    print("Testing adc Futek sensor (100 Nm max). Press CTRL-C to finish now.")
    loop = SoftRealtimeLoop(dt = 0.00333333333, report=True)
    for i, t in enumerate(loop):
        adc.update()
        if i%100==0:
            print ("τ: %.2f Nm"%(adc.get_torque()))

if __name__ == '__main__':
    with Big100NmFutek(csv_file_name="junk.csv") as adc:
        measure_torque(adc)
