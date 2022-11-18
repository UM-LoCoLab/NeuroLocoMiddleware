from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from UdpBinarySynch import UdpBinarySynchA
import numpy as np



print("A1")
# context = zmq.Context()
# print("A2")


synch = UdpBinarySynchA(
    recv_IP="127.0.0.1",
    recv_port=5557,
    send_IP="127.0.0.1", 
    send_port=5558)

for t in SoftRealtimeLoop(0.001, report=True):
    print(synch.update(np.array([42.1, t*1000])))