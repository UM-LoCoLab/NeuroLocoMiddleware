from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
import zmq
import time
import numpy as np
#∅mq
#ØMQ
#🎄
print("A1")
context = zmq.Context()
print("A2")
socketA = context.socket(zmq.PUB)
print("A3")
socketA.bind("tcp://*:5557")
print("A4")
socketB = context.socket(zmq.SUB)
print("A5")
socketB.connect("tcp://localhost:5558")

socketB.setsockopt(zmq.SUBSCRIBE, b'B')
print("A6")
# poller = zmq.Poller()
# poller.register(socketB, zmq.POLLIN)

print("A7")
my_count = -42

for t in SoftRealtimeLoop(0.001, report=True):
    print("A8", t)
    # socks = dict(poller.poll(1))

    # print("A9", socks)
    # if socketB in socks and socks[socketB] == zmq.POLLIN:
    
    print("A10")
    try: 
        message = socketB.recv(zmq.NOBLOCK)
        print("A11")
        print("Received:", message)
        my_count=int(message[1:4])
        my_vecB = np.frombuffer(message[4:])
        print("A12", my_count, my_vecB)

        #  Send reply back to client
        socketA.send(b"A%03d"%my_count)
        print("A13", b"A%03d"%my_count)
    except KeyboardInterrupt:
        socketB.close()
        break
    except zmq.error.Again:
        pass
    print("A14")