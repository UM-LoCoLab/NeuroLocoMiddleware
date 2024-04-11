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

import time

RECORD_VICON = False
    
class BertecGUI(KivyApp):
    """
    Bertec GUI class, defines all GUI properties. 
    Jiefu Zhang 2023. 
    
    Updated 2024 - Kevin Best

    Updated 04/2024 - Katharine Walters (update log name with timestamp, make Vicon optional)
    """    
    def __init__(self, test_duration=60*6, log_name='log', record_vicon = False):
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

        self.bertecObj = Bertec()
        self.bertecObj.start()
        print('Bertec communication set up')

        self.log = Logger(log_name)
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
        
        pause_button = Button(text ="Pause",
                   font_size ="50sp",
                   background_color =(0/255,0/255, 0/255, 1),
                   color =(1, 1, 1, 1),
                   size =(32, 32),
                   size_hint =(1, 1))
        
        stop_button = Button(text ="Stop",
                   font_size ="50sp",
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

if __name__ == '__main__':
    print("Starting Bertec GUI")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S") 
    log_name = "./BertecSelfPacedTest_" + timestamp

    bertecGUI = BertecGUI(log_name = log_name, record_vicon = RECORD_VICON)
    bertecGUI.run()