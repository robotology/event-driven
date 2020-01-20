# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Massimiliano Iacono
         Sim Bamford

This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Use kivy to create an app which can receive data dicts as imported by bimvee
importAe, and allow synchronised playback for each of the contained channels and datatypes. 
"""
# standard imports 
import matplotlib.pyplot as plt
import numpy as np

# Optional import of tkinter allows setting of app size wrt screen size
try:
    import tkinter as tk
    from kivy.config import Config
    root = tk.Tk()
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    Config.set('graphics', 'position', 'custom')
    Config.set('graphics', 'left', int(screen_width/8))
    Config.set('graphics', 'top',  int(screen_width/8))
    Config.set('graphics', 'width',  int(screen_width/4*3))
    Config.set('graphics', 'height',  int(screen_height/4*3))
    #Config.set('graphics', 'fullscreen', 1)
except ModuleNotFoundError:
    pass

# kivy imports
from kivy.app import App
from kivy.uix.slider import Slider
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics.texture import Texture
from kivy.properties import ObjectProperty
from kivy.uix.popup import Popup
from kivy.properties import StringProperty, NumericProperty, ListProperty, BooleanProperty
from kivy.properties import DictProperty, ReferenceListProperty
from kivy.metrics import dp

# local imports (from bimvee)
try:
    from visualiser import VisualiserDvs, VisualiserFrame, VisualiserPose6q
    from timestamps import getLastTimestamp
except ModuleNotFoundError:
    from libraries.bimvee.visualiser import VisualiserDvs, VisualiserFrame, VisualiserPose6q
    
    
class ErrorPopup(Popup):
    label_text = StringProperty(None)


class WarningPopup(Popup):
    label_text = StringProperty(None)


class TextInputPopup(Popup):
    label_text = StringProperty(None)

class LoadDialog(FloatLayout):
    load = ObjectProperty(None)
    cancel = ObjectProperty(None)
    load_path = StringProperty(None)

class Viewer(Image):
    labels = ListProperty()
    data = ObjectProperty(force_dispatch=True)
    need_init = BooleanProperty(True)
    dsm = ObjectProperty(None, allownone=True)
    colorfmt = 'luminance'
    allow_stretch = BooleanProperty(True)
    
    def on_dsm(self, instance, value):
        if self.dsm is not None:
            self.colorfmt = self.dsm.get_colorfmt()
            x, y = self.dsm.get_dims()
            buf_shape = (dp(x), dp(y))
            self.texture = Texture.create(size=buf_shape, colorfmt=self.colorfmt)
            self.is_x_flipped = False
            self.is_y_flipped = False
            self.need_init = False
            
    def __init__(self, **kwargs):
        super(Viewer, self).__init__(**kwargs)
        self.cm = plt.get_cmap('tab20')
        self.is_x_flipped = False
        self.is_y_flipped = False
        self.texture = None
        Clock.schedule_once(self.init, 0)
        
    def init(self, dt):
        self.data = np.zeros((1, 1), dtype=np.uint8)

    def on_data(self, instance, value): 
        self.update_data(self.data)
    
    def update_data(self, data):
        if self.need_init:
            self.on_dsm(None, None)
        if self.texture is not None:
            self.texture.blit_buffer(data.tostring(), bufferfmt="ubyte", colorfmt=self.colorfmt)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = plt.imread('graphics/missing.jpg')
        else:
            self.data = self.dsm.get_frame(time_value, time_window, children=self.children)
        
    def get_frame_at_time(self, time, time_window, dualImg):
        if self.dsm is None:
            return plt.imread('graphics/missing.jpg')
        else:
            return self.dsm.get_frame(time, time_window, children=self.children)
   
class DataController(GridLayout):
    ending_time = NumericProperty(.0)
    file_path_or_name = StringProperty('')
    data_dict = DictProperty({}) # A bimvee-style container of channels
    
    def __init__(self, **kwargs):
        super(DataController, self).__init__(**kwargs)
    
    def update_children(self):
        for child in self.children:
            child.get_frame(self.time_value, self.time_window)
       
    def add_viewer_and_resize(self, visualiser):
        new_viewer = Viewer()
        new_viewer.dsm = visualiser
        self.add_widget(new_viewer)
        print(len(self.children))
        print(int(np.ceil(np.sqrt(len(self.children)))))
        print()
        self.cols = int(np.ceil(np.sqrt(len(self.children))))

    def add_viewer_for_each_channel_and_data_type(self, in_dict):
        if isinstance(in_dict, list):
            for in_dict_element in in_dict:
                self.add_viewer_for_each_channel_and_data_type(in_dict_element)
        elif isinstance(in_dict, dict):
            for key_name in in_dict.keys():
                if isinstance(in_dict[key_name], dict):
                    if 'ts' in in_dict[key_name]:
                        print('Creating a new viewer, of type: ' + key_name)
                        if key_name == 'dvs':
                            self.add_viewer_and_resize(VisualiserDvs(in_dict[key_name]))
                        elif key_name == 'frame':
                            self.add_viewer_and_resize(VisualiserFrame(in_dict[key_name]))
                        elif key_name == 'pose6q':
                            self.add_viewer_and_resize(VisualiserPose6q(in_dict[key_name]))
                        else:
                            print('datatype not supported: ' + key_name)
                    else: # recurse through the sub-dict
                        self.add_viewer_for_each_channel_and_data_type(in_dict[key_name])
    
    def on_data_dict(self, instance, value):
        self.ending_time = float(getLastTimestamp(self.data_dict)) # timer is watching this
        while len(self.children) > 0:
            self.remove_widget(self.children[0])
            print('Removed an old viewer; num remaining viewers: ' + str(len(self.children)))
        self.add_viewer_for_each_channel_and_data_type(self.data_dict)

    def dismiss_popup(self):
        self._popup.dismiss()

    def show_load(self):
        content = LoadDialog(load=self.load,
                             cancel=self.dismiss_popup)
        self._popup = Popup(title="Load file", content=content,
                            size_hint=(0.9, 0.9))
        self._popup.open()

    def load(self, path, selection):
        self.dismiss_popup()
        from libraries.bimvee.importRpgDvsRos import importRpgDvsRos

        template = {
            'davis': {
                'dvs': '/dvs/events',
                'frame': '/dvs/image_raw',
            },
            'extra': {
                'frame': '/dvs/depthmap',
                'pose6q': '/dvs/pose'
            }
        }

        self.data_dict = importRpgDvsRos(filePathOrName=selection[0], template=template, )



#    def dismiss_popup(self):
#        if self._popup is not None:
#            self._popup.dismiss()
    
class TimeSlider(Slider):
    def __init__(self, **kwargs):
        super(TimeSlider, self).__init__(**kwargs)
        self.clock = None
        self.speed = 1
    
    def increase_slider(self, dt):
        self.value = min(self.value + dt / self.speed, self.max)
        if self.value >= self.max:
            if self.clock is not None:
                self.clock.cancel()
    
    def decrease_slider(self, dt):
        self.value = max(self.value - dt / self.speed, 0.0)
        if self.value <= 0.0:
            if self.clock is not None:
                self.clock.cancel()
    
    def play_pause(self):
        if self.clock is None:
            self.clock = Clock.schedule_interval(self.increase_slider, 0.001)
        else:
            if self.clock.is_triggered:
                self.clock.cancel()
            else:
                self.clock.cancel()
                self.clock = Clock.schedule_interval(self.increase_slider, 0.001)

    def pause(self):
        if self.clock is not None:
            self.clock.cancel() 
 
    def play_forward(self):
        if self.clock is not None:
            self.clock.cancel() 
        self.clock = Clock.schedule_interval(self.increase_slider, 0.001)
 
    def play_backward(self):
        if self.clock is not None:
            self.clock.cancel() 
        self.clock = Clock.schedule_interval(self.decrease_slider, 0.001)
 
    def stop(self):
        if self.clock is not None:
            self.clock.cancel()
            self.set_norm_value(0)
    
    def reset(self):
        self.value = 0

    def step_forward(self):
        #self.increase_slider(self.time_window)
        self.increase_slider(0.016)

    def step_backward(self):
        #self.decrease_slider(self.time_window)
        self.decrease_slider(0.016)
                
class RootWidget(BoxLayout):
    def __init__(self, **kwargs):
        super(RootWidget, self).__init__(**kwargs)


class Ntupleviz(App):
    def build(self):
        return RootWidget()

if __name__ == '__main__':
    Ntupleviz().run()

