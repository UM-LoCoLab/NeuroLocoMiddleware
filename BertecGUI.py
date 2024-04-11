from kivy.app import App as KivyApp
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout    
from kivy.uix.image import Image    
from kivy.core.window import Window
from kivy.uix.label import Label 
from kivy.clock import Clock

from datetime import datetime
from BertecMan import Bertec
from logger import Logger
from ViconMan import Vicon

from typing import Any, Callable, List, Union
import csv
import logging
from logging.handlers import RotatingFileHandler

import time

RECORD_VICON = False
TEST_DURATION = 60*6       # test duration in seconds
    
class BertecGUI(KivyApp):
    """
    Bertec GUI class, defines all GUI properties. 
    Jiefu Zhang 2023. 
    
    Updated 2024 - Kevin Best

    Updated 04/2024 - Katharine Walters (update log name with timestamp, make Vicon optional)
    """    
    def __init__(self, test_duration=TEST_DURATION, log_name='log', record_vicon = RECORD_VICON, ip_address = '127.0.0.1'):
        super().__init__()
        self.test_duration = test_duration
        self.log_name = log_name[2:-1]
        self._test_running = False
        self._test_start_time = time.time()
        self.last_update_time = time.time()
        self._gui_start_time = time.time()
        self.started = False
        self.completed = False
        self.record_vicon = record_vicon

        self.speed0 = 0.6   # Initial speed (m/s)
        self.acc = 0.25   # Acceleration (m/s^2)

        self.bertecObj = Bertec(viconPC_IP = ip_address)
        self.bertecObj.start()
        print('Bertec communication set up')

        self.log = SelfPacedLogger(log_name)
        self.log.add_attributes(self, ['test_time','gui_time'])
        self.log.add_attributes(self.bertecObj, ['distance','speed'])

        if self.record_vicon == True:
            self.vicon = Vicon()

    def build(self):
        Window.clearcolor = (202/255, 217/255, 206/255, 1)
        
        superBox = BoxLayout(orientation='vertical')
        MiddleBox = BoxLayout(orientation='horizontal', spacing=10, size_hint=(1, 0.2), padding=10)
        startStopBox = BoxLayout(orientation='horizontal', spacing=10, size_hint=(1, 0.2), padding=10)

        btn2 = Button(text ="–",
                   font_size ="100sp",
                   background_color =(1, 0, 0, 0.9),
                   color =(1, 1, 1, 1),
                   size =(32, 32),
                   size_hint =(1, 0.4))
        
        btn1 = Button(text ="+",
                   font_size ="100sp",
                   background_color =(22/255, 231/255, 22/255, 0.9),
                   color =(1, 1, 1, 1),
                   size =(32, 32),
                   size_hint =(1, 0.4))
        
        startButton = Button(text ="Start",
                   font_size ="50sp",
                   background_color =(0/255,0/255, 0/255, 1),
                   color =(1, 1, 1, 1),
                   size =(32, 32),
                   size_hint =(1, 1))
        
        pause_button = Button(text ="Pause \n Test",
                   font_size ="30sp",
                   background_color =(0/255,0/255, 0/255, 1),
                   color =(1, 1, 1, 1),
                   size =(32, 32),
                   size_hint =(1, 1))
        
        stop_button = Button(text ="Stop \n Treadmill",
                   font_size ="40sp",
                   background_color =(0/255,0/255, 0/255, 1),
                   color =(1, 1, 1, 1),
                   size =(32, 32),
                   size_hint =(1, 1))
 
        btn1.bind(on_press = self.increment_button_callback)
        btn2.bind(on_press = self.decrement_button_callback)
        startButton.bind(on_press = self.start_button_callback)
        pause_button.bind(on_press = self.pause_button_callback)
        stop_button.bind(on_press = self.stop_button_callback)

        self.timer_text = Label(text="Time remaining: %.2f s" % self.get_time_remaining(),
                           font_size ="40sp",
                           color=(0,0,0,1),
                           halign='left')

        Locolab_logo = Image(source='locolab_logo.jpg', size_hint=(.2, 1))
        MiddleBox.add_widget(Locolab_logo)
        MiddleBox.add_widget(self.timer_text)

        superBox.add_widget(btn1)
        superBox.add_widget(MiddleBox)
        superBox.add_widget(btn2)

        startStopBox.add_widget(startButton)
        startStopBox.add_widget(stop_button)
        startStopBox.add_widget(pause_button)
        superBox.add_widget(startStopBox)

        dt = 0.1
        Clock.schedule_interval(self.update_GUI, dt)
        if self.record_vicon == True:
            self.vicon.start_recording(self.log_name)

        return superBox

    def update_GUI(self, event): 
        time_now = time.time()
        dt = time_now - self.last_update_time
        if not self._test_running:
            self._test_start_time = self._test_start_time + dt

        self.timer_text.text = "Time remaining: %.2f s" % self.get_time_remaining()
        if self.get_time_remaining() == 0:
            self.bertecObj.write_command(0.0, 0.0, accR=self.acc, accL=self.acc)
            self.completed = True

        self.log.data()

        self.last_update_time = time_now
        pass

    def get_time_remaining(self):
        return max(0, self.test_duration - (time.time() - self._test_start_time))
    
    @property
    def test_time(self):
        return self.test_duration - self.get_time_remaining()
    
    @property 
    def gui_time(self):
        return time.time() - self._gui_start_time

    def on_start(self):
        pass

    def on_stop(self):
        self.bertecObj.write_command(0, 0, accR=self.acc, accL=self.acc)
        self.bertecObj.stop()
        self.completed = True
        print('Bertec communication closed')
 
    # callback function tells when button pressed
    def increment_button_callback(self, event):
        self.incremental_speed_change(1, 0.05)

    def decrement_button_callback(self, event):
        self.incremental_speed_change(-1, 0.05)

    def start_button_callback(self, event):
        self._test_running = True
        self.started = True
        self.bertecObj.write_command(self.speed0, self.speed0, accR=self.acc, accL=self.acc)

    def stop_button_callback(self, event):
        self.bertecObj.write_command(0.0, 0.0, accR=self.acc, accL=self.acc)

    def pause_button_callback(self, event):
        self._test_running = False
        self.bertecObj.write_command(0.0, 0.0, accR=self.acc, accL=self.acc)

    def incremental_speed_change(self, direction, delta):
        """direction \in {-1, 1} to say whether to increase or decrease speed. """
        speedL, speedR = self.bertecObj.get_belt_speed()         # TODO: The speed of bertec is not a constant
        speedAvg = (speedL + speedR) / 2
        newSpeed = speedAvg + delta*direction
        self.bertecObj.write_command(newSpeed, newSpeed) 

        print("Speed changed from %.2f m/s, is now: %.2f m/s" % (speedAvg, newSpeed), end='\n')



class SelfPacedLogger(logging.Logger):
    """
    Logger class is a class that logs attributes from a class to a csv file

    Methods:
        __init__(self, container: object, file_path: str, logger: logging.Logger = None) -> None
        log(self) -> None
    """

    def __init__(
        self,
        file_path: str = "./osl",
        log_format: str = "[%(asctime)s] %(levelname)s: %(message)s",
    ) -> None:

        self._file_path: str = file_path + ".log"

        self._containers: list[Union[object, dict[Any, Any]]] = []
        self._attributes: list[list[str]] = []

        self._file = open(file_path + ".csv", "w", newline='')
        self._writer = csv.writer(self._file)

        self._log_levels = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARNING": logging.WARNING,
            "ERROR": logging.ERROR,
            "CRITICAL": logging.CRITICAL,
        }

        super().__init__(__name__)
        self.setLevel(logging.DEBUG)

        self._std_formatter = logging.Formatter(log_format)

        self._file_handler = RotatingFileHandler(
            filename=self._file_path,
            mode="w",
            maxBytes=0,
            backupCount=10,
        )
        self._file_handler.setLevel(level=logging.DEBUG)
        self._file_handler.setFormatter(fmt=self._std_formatter)

        self._stream_handler = logging.StreamHandler()
        self._stream_handler.setLevel(level=logging.INFO)
        self._stream_handler.setFormatter(fmt=self._std_formatter)

        self.addHandler(hdlr=self._stream_handler)
        self.addHandler(hdlr=self._file_handler)

        self._is_logging = False

    def __repr__(self) -> str:
        return f"Logger"

    def set_file_level(self, level: str = "DEBUG") -> None:
        """
        Sets the level of the logger

        Args:
            level (str): Level of the logger
        """
        if level not in self._log_levels.keys():
            self.warning(msg=f"Invalid logging level: {level}")

        self._file_handler.setLevel(level=self._log_levels[level])

    def set_stream_level(self, level: str = "INFO") -> None:
        """
        Sets the level of the logger

        Args:
            level (str): Level of the logger
        """
        if level not in self._log_levels.keys():
            self.warning(msg=f"Invalid logging level: {level}")

        self._stream_handler.setLevel(level=self._log_levels[level])

    def add_attributes(
        self, container, attributes
    ) -> None:
        """
        Adds class instance and attributes to log

        Args:
            container (object, dict): Container can either be an object (instance of a class)
                or a Dict containing the attributes to be logged.
            attributes (list[str]): List of attributes to log
        """
        self._containers.append(container)
        self._attributes.append(attributes)

    def data(self) -> None:
        """
        Logs the attributes of the class instance to the csv file
        """
        header_data = []
        data = []

        if not self._is_logging:
            for container, attributes in zip(self._containers, self._attributes):
                for attribute in attributes:
                    if type(container) is dict:
                        if "__main__" in container.values():
                            header_data.append(f"{attribute}")
                        else:
                            header_data.append(f"{container}:{attribute}")
                    else:
                        if type(container).__repr__ is not object.__repr__:
                            header_data.append(f"{container}:{attribute}")
                        else:
                            header_data.append(f"{attribute}")

            self._writer.writerow(header_data)
            self._is_logging = True

        for container, attributes in zip(self._containers, self._attributes):
            if isinstance(container, dict):
                for attribute in attributes:
                    data.append(container.get(attribute))
            else:
                for attribute in attributes:
                    data.append(getattr(container, attribute))

        self._writer.writerow(data)
        self._file.flush()

    def close(self) -> None:
        """
        Closes the csv file
        """
        self._file.close()


if __name__ == '__main__':
    print("Starting Bertec GUI")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S") 
    log_name = "./BertecSelfPacedTest_" + timestamp

    bertecGUI = BertecGUI(log_name = log_name)
    bertecGUI.run()