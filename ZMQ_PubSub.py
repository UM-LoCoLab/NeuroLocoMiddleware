"""
This module implements a publisher/subscriber networking protocol using ZMQ. 
Its development was motivated by the need to share timestamps between two python programs on different PCs. 
My current usage is to have the OSL as a publisher of its current log timestamp. 
A script on a Vicon PC will subscribe to that timestamp to embed sync behavior in a vicon log. 

See the test_sub() and test_pub() methods for example usages. 

Requires pyzmq

Kevin Best, 8/17/2023
"""
import zmq

class Subscriber():
    def __init__(self, publisher_ip = 'localhost', publisher_port = "5556", 
                 timeout_ms = 100, topic_filter = '', get_latest_only = True) -> None:
        """
        Instantiate a subscriber object. Requires the IP address of the publisher, the port of the publisher, an optional timeout flag, optional topic filter, and a bool to ask for only the latest value if there are more than one in the buffer. 
        """
        # Socket to talk to server
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
        if get_latest_only:
            self.socket.setsockopt(zmq.CONFLATE, 1) 
        self.socket.connect(("tcp://" + publisher_ip + ":%s") % publisher_port)
        self.timeout_ms = timeout_ms
        self.encoding =  'UTF-8'
    
    def get_message(self) -> (str, str, bool):
        """
        Checks for a message from the subscribed topics. If no message is available in the timeout, it'll return blank topic and message. The msg_received flag will also be false. 
        Returns topic, message, msg_received flag
        """
        msg_received = self.socket.poll(self.timeout_ms)
        if msg_received == 0:
            message_decoded = ''
            topic_decoded = ''
        else:
            string = self.socket.recv(zmq.NOBLOCK)
            topic, message = string.split()
            topic_decoded = str(topic, self.encoding)
            message_decoded = str(message, self.encoding)
        return topic_decoded, message_decoded, msg_received


class Publisher():
    """ 
    Instantiates a publisher object. Only required input is a port. Any subscriber on the network can subscribe to this topic via the IP address of the publisher. 
    """
    def __init__(self, port = "5556") -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)

        self.socket.bind("tcp://*:%s" % port)

    def publish(self, topic, message) -> None:
        """
        Publish a message on a specified topic. Note that topic names CANNOT have spaces in them. 
        """
        assert " " not in topic, "topic name cannot have spaces!"
        self.socket.send_string(topic + " " + message)


def testSub():
    """
    Import this method and run in a thread to test the subscriber behavior. Once you start a publisher, you should see the data parsed. 
    """
    sub = Subscriber()
    while(1):
        topic, string, event = sub.get_message()
        if event:
            print("Topic: " + topic + " String: " + string)

def testPub():
    """
    Import this method and run in a thread to see publisher behavior. You should see whatever data you sent here appear in a subscriber. 
    """
    pub = Publisher()
    i = 0

    while(1):
        pub.publish("Test_Publisher_Topic_1","%d" % i)
        i+=1

