# NeuroLoco Middleware
This repo contains useful python tools for running experiments. It is jointly maintained by the Locomotor Control Systems Lab and the Neurobionics Lab at the University of Michigan. 

## Contents Overview
### Code Tools
- `SoftRealtimeLoop.py` - A class designed to allow clean exits from infinite loops with the potential for post-loop cleanup operations executing. There is a parallel version in the Open-Source Leg library that should be identical. 
- `StatProfiler.py` - A class that helps time the duration of sections of code. It includes both matlab-style tic-toc timing and function decoration timing.

### Communication 
- `UDPBinarySynch.py` - Classes used to synchonize code between two computers via UDP.
- `VariableUpdater.py` - A class used to pass variable information across processes and computers via zmq.
- `ZMQ_PubSub.py` - A wrapper class for handling publish/subscribe messages between processes and computers via zmq.
- `ZmqBinarySynch.py` - Similar to `UDPBinarySynch.py`, but implemented in zmq.
  
### Equipment Managing/Interfacing 
- `BertecGUI.py` - A GUI that provides a user interface for the Bertec Treadmill with simple increase/decrease speed buttons. A typical use case is to run this on a tablet and allow a participant to change the treadmill speed at will, similar to a standard exercise treadmill.
- `BertecMan.py` - A class to control the Bertec treadmill from python.
- `ViconMan.py` - A class used to start and stop Vicon recordings from python.

### Math
- `LinearFilter.py` - A discrete state-space linear filter.

### Sensor Interfacing
- `AdcManager.py` - A wrapper class for using the adafruit ads1115 ADC module.
- `AhrsManager.py` - A wrapper class to handle interfacing with the Microstrain AHRS IMU. NOTE: The implementation in the Open-Source Leg library may be more up to date.



## Contributing
To contribute, please fork the repo, make your change, and submit a pull request. 

## Graveyard :skull:
### Modules
These files used to exist but have since been deleted. In some world where you need them, here's their final commit:
- [`ActPackMan.py`](https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/a6b5e73d415da1fc3d97945d636fbe1589187140/ActPackMan.py#L1) This wrapper class was written for the Dephy actpacks. It has been superseded by the implementation in the [OpenSourceLeg library](https://github.com/neurobionics/opensourceleg). 
- [`EB51Man.py`](https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/a6b5e73d415da1fc3d97945d636fbe1589187140/EB51Man.py#L1) A wrapper class for the Dephy ExoBoots, last used by Katharine Walters. Included belt tightening, nonlinear gear ratio, and other booty features.
- [`FileGlobal.py`](https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/a6b5e73d415da1fc3d97945d636fbe1589187140/FileGlobal.py#L1) Last used by Prof. Gray Thomas. Not sure what this does. 
- [`MBLUEmod.py`](https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/a6b5e73d415da1fc3d97945d636fbe1589187140/MBLUEmod.py#L1) This looks like an initial software library for the MBLUE exos but it isn't used anymore. Latest commit was from Prof. Gray Thomas. There are likely other places to start if you're controlling MBLUE. It also seems like it has some test bench functionality, if you're curious. 
- [`SysID.py`](https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/a6b5e73d415da1fc3d97945d636fbe1589187140/SysID.py#L1) Yet again from the illustrious Prof. Gray Thomas, this script generates a chirp signal to be used in a frequency-based sys id. 
- [`frequency_analysis.py`](https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/a6b5e73d415da1fc3d97945d636fbe1589187140/frequency_analysis.py#L1) This script makes a bode plot based on measured data. Talk to Prof. Gray Thomas. 

### Tests
We deleted many of the test scripts associated with the above files. If you go to the commits referenced above, they will still exist. 
