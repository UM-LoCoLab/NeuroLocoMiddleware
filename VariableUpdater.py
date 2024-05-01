import zmq
import threading
import time

class VariableUpdater:
    """
    This class has a floating value called 'variable' that can be updated by a 
    separate process. The class listens for updates to the variable and updates
    it accordingly. Multiple instances of this class can be created to listen
    for updates to different variables as long as they have different 
    subscription ports.
    """
    def __init__(self, initial_value:float, sub_port:str="5551", 
                       update_interval:int=1):
        """
        Initialize the VariableUpdater object with the initial value of the
        variable and the subscription port to listen for updates.   

        Args:
            initial_value (float): The initial value of the variable
            sub_port (str, optional): The subscription port to listen for updates.
                Defaults to "5551".
            update_interval (int, optional): The interval at which to check for
                updates in Hz. Defaults to 1 Hz.
        """
        # Initialize the variable and update interval
        self.variable = initial_value
        self.update_interval = update_interval
        # Create a ZeroMQ context to generate a subscribe socket
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect(f"tcp://127.0.0.1:{sub_port}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        # Start a thread to listen for updates
        thread = threading.Thread(target=self.listen_for_updates)
        thread.daemon = True
        thread.start()
    
    def listen_for_updates(self):
        """
        This method listens for updates to the variable and updates it
        """
        while True:
            try:
                # Receive a Python object
                message = self.subscriber.recv_pyobj(flags=zmq.NOBLOCK)  
                # Ensure it's a float
                if isinstance(message, float):  
                    self.variable = message
            # If we don't receive a message, continue loop
            except zmq.Again:
                pass 
            # Prevent busy waiting
            time.sleep(1/self.update_interval) 

def send_update(new_value, pub_port="5551"):
    """
    Send an update to the variable to the subscriber listening on the specified
    port.
    """
    # Create a ZeroMQ context to generate a publish socket
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://*:{pub_port}")
    # Wait for the subscriber to connect
    time.sleep(1)  
    try:
        # Send a Python object
        publisher.send_pyobj(new_value)  
    # In case we get an error, close the publisher and context
    finally:
        publisher.close()
        context.term()