from FindLibrariesWarning import *
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import UdpBinarySynch as ubs
import numpy as np



# print("A1")
# context = zmq.Context()
# print("A2")


synch = ubs.UdpBinarySynchA(
    recv_IP="192.168.1.157",
    recv_port=5558,
    send_IP="192.168.1.216", 
    send_port=5557)

for t in SoftRealtimeLoop(0.000001, report=True):
    print(synch.update(np.array([])))