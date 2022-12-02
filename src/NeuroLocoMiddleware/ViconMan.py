import socket
from random import randint
from time import sleep


class Vicon:
    """
    A class for managing starting and stopping Vicon recordings
    over the network. 
    To use this class, make sure you've armed vicon and enabled network triggers.
    See this page for more info: https://docs.vicon.com/display/Nexus213/Automatically+start+and+stop+capture
    Kevin Best 10/22
    """
    def __init__(self, viconPC_IP = '141.212.77.16', viconPC_port = 30,
                viconPath = 'E:'):

        self.destinationIP = viconPC_IP
        self.destinationPort = viconPC_port
        self.viconPath = viconPath
        self.delayPriorToRecord_ms = 1
        self.fileName = ''
        self.fileDescription = ''

        # Setup UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 

    def start_recording(self, fileNameIn: str, fileDescription: str = 'A vicon recording'):
        """
        Send command to vicon to start file recording.
        Requires 2 inputs:
            fileNameIn: File name to be used on the vicon PC
            fileDescription: Any notes you want to add to your file. Fills the description field on vicon 
        """
        msg = self._assemble_payload_start(fileNameIn, fileDescription)
        self.sock.sendto(msg, (self.destinationIP, self.destinationPort))

    def stop_recording(self):
        msg = self._assemble_payload_stop()
        self.sock.sendto(msg, (self.destinationIP, self.destinationPort))

    def _assemble_payload_start(self, fileNameIn, fileDescription):
        """
        Creates the proper XML string to trigger vicon. 
        More documentation available here: 
           https://docs.vicon.com/pages/viewpage.action?pageId=152010925
        """

        # Update self
        self.fileName = fileNameIn
        self.fileDescription = fileDescription

        # Construct the string
        notes = 'Vicon triggered from python over UDP'
        cmdHeader = '\n<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n<CaptureStart>\n'
        nameLine = '<Name VALUE="{}"/>\n'.format(self.fileName)
        notesLine = '<Notes VALUE="{}"/>\n'.format(notes)
        descriptionLine = '<Description VALUE="{}"/>\n'.format(self.fileDescription)
        databasePathLine = '<DatabasePath VALUE="{}"/>\n'.format(self.viconPath)
        delayLine = '<Delay VALUE="{}"/>\n'.format(self.delayPriorToRecord_ms)
        packetIDline = '<PacketID VALUE="{}"/>\n'.format(randint(1,2**16))
        suffixLine = '</CaptureStart>'
        fullPayloadString = cmdHeader + nameLine + notesLine + descriptionLine + databasePathLine + delayLine + packetIDline + suffixLine
        # print(fullPayloadString)

        # Convert string to utf-8 bytes string to send over network. 
        return bytes(fullPayloadString, "utf-8")

    def _assemble_payload_stop(self):
        """
        Creates the proper XML string to stop vicon. 
        More documentation available here: 
           https://docs.vicon.com/pages/viewpage.action?pageId=152010925
        """

        cmdHeader = '\n<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n<CaptureStop RESULT="SUCCESS">\n'
        nameLine = '<Name VALUE="{}"/>\n'.format(self.fileName)
        databasePathLine = '<DatabasePath VALUE="{}"/>\n'.format(self.viconPath)
        delayLine = '<Delay VALUE="{}"/>\n'.format(self.delayPriorToRecord_ms)
        packetIDline = '<PacketID VALUE="{}"/>\n'.format(randint(1,2**16))
        suffixLine = '</CaptureStop>'
        fullPayloadString = cmdHeader + nameLine + databasePathLine + delayLine + packetIDline + suffixLine
        # print(fullPayloadString)

        # Convert string to utf-8 bytes string to send over network. 
        return bytes(fullPayloadString, "utf-8")


if __name__ == '__main__':
    vicon = Vicon()
    vicon.start_recording('File1')
    sleep(5)
    vicon.stop_recording()