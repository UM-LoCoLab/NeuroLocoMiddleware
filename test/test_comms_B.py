#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#
from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
import zmq
import numpy as np


print("B1")
context = zmq.Context()

print("B2")
socketB = context.socket(zmq.PUB)
print("B3")
socketB.bind("tcp://*:5558")
print("B4")
#  Socket to talk to server
print("Connecting to hello world server…")
socketA = context.socket(zmq.SUB)
socketA.setsockopt(zmq.SUBSCRIBE, b'A')
print("B5")
socketA.connect("tcp://localhost:5557")
print("B6")
# poller = zmq.Poller()
# poller.register(socketA, zmq.POLLIN)
print("B7")

my_count = 0

for t in SoftRealtimeLoop(0.001, report=True):
    print("B8", t)
    # socks = dict(poller.poll(0))

    # print("B9", socks)
    # if socketA in socks and socks[socketB] == zmq.POLLIN:
    try: 
        print("B10")
        message = socketA.recv(zmq.NOBLOCK)
        print("B11")
        print("Received:", message)
        if int(message[1:4])==my_count:
            my_count+=1
            if my_count>=1000:
                my_count=0
    except KeyboardInterrupt:
        socketA.close()
        break
    except zmq.error.Again:
        pass
    print("B12", my_count)

    vecB = np.array([[0.34,0.222, 0.5]])
    socketB.send(b"B%03d%s"%(my_count, vecB.tobytes()))
    print("B13", b"B%03d%s"%(my_count, vecB.tobytes()))