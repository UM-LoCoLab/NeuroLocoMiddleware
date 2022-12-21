from FindLibrariesWarning import * 
from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan
import numpy as np
import time
import csv

def calibrate(dev, writer):
    print("Press CTRL-C to finish.")
    dev.set_current_gains()
    loop = SoftRealtimeLoop(dt = 1/277, report=True, fade=0.01)
    for i, t in enumerate(loop):
        dev.update()
        dev.τ= 0.1
        ank_ang = dev.act_pack.ank_ang * 2*np.pi / pow(2,14) * 180/np.pi # degrees 
        mot_ang = dev.act_pack.mot_ang

        writer.writerow([ank_ang, mot_ang])
        
        if i%100==0:
            print('Ankle angle: ', ank_ang)
    

def main():
    with ActPackMan('/dev/ttyActPackA', gear_ratio=1, updateFreq=1000) as dev:
        f = open('calibration_data.csv', 'w') 
        writer = csv.writer(f)  
        calibrate(dev, writer)
    f.close()
    print("calibration") 

if __name__ == '__main__':
    main()
