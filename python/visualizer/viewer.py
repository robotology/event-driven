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

Collection of possible Viewers that can be spawned in the main app window. Each one will have
a visualiser which is responsible for the data management and retrieval.
"""

from imageio import imread
import numpy as np
from kivy.graphics.texture import Texture
from kivy.uix.widget import Widget
from kivy.properties import BooleanProperty, ObjectProperty, StringProperty, ListProperty, DictProperty
from kivy.metrics import dp
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock


class BoundingBox(Widget):
    def __init__(self, bb_color, x, y, width, height, **kwargs):
        super(BoundingBox, self).__init__(**kwargs)
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.bb_color = bb_color


class LabeledBoundingBox(BoundingBox):
    obj_label = StringProperty('')

    def __init__(self, bb_color, x, y, width, height, label, **kwargs):
        super(LabeledBoundingBox, self).__init__(bb_color, x, y, width, height, **kwargs)
        self.obj_label = '{:d}'.format(int(label))


class Viewer(BoxLayout):
    data = DictProperty(force_dispatch=True)
    visualisers = ListProperty([], allownone=True)
    flipHoriz = BooleanProperty(False)
    flipVert = BooleanProperty(False)
    settings = DictProperty({}, allownone=True)
    colorfmt = 'luminance'

    def on_visualisers(self, instance, value):
        if self.visualisers is not None and self.visualisers:
            for v in self.visualisers: # TODO manage cases with multiple of below data_types
                if v.data_type in ['dvs', 'frame', 'pose6q', 'point3']:
                    self.colorfmt = v.get_colorfmt()
                    self.data_shape = v.get_dims()
                    buf_shape = (dp(self.data_shape[0]), dp(self.data_shape[1]))
                    self.image.texture = Texture.create(size=buf_shape, colorfmt=self.colorfmt)

    def __init__(self, **kwargs):
        super(Viewer, self).__init__(**kwargs)
        # self.image = Image()
        # self.add_widget(self.image)
        from matplotlib.pyplot import get_cmap
        self.cm = get_cmap('tab20')
        #self.image.texture = None
        Clock.schedule_once(self.init, 0)

    def init(self, dt):
        self.data = {}

    def on_data(self, instance, value):
        for data_type in self.data.keys():
            if data_type in ['dvs', 'frame', 'pose6q', 'point3']:
                self.update_image(self.data[data_type])
            elif data_type in ['boundingBoxes']:
                self.update_b_boxes(self.data[data_type])

    def update_image(self, data):
        if self.image.texture is not None:
            size_required = self.data_shape[0] * self.data_shape[1] * (1 + (self.colorfmt == 'rgb') * 2)
            if not isinstance(data, np.ndarray):
                data = np.zeros((self.data_shape[0], self.data_shape[1], 3), dtype=np.uint8)
            if data.size >= size_required:
                try:
                    if self.flipHoriz:
                        data = np.flip(data, axis=1)
                    if not self.flipVert:  # Not, because by default, y should increase downwards, following https://arxiv.org/pdf/1610.08336.pdf
                        data = np.flip(data, axis=0)
                except AttributeError:
                    pass  # It's not a class that allows flipping
                self.image.texture.blit_buffer(data.tostring(), bufferfmt="ubyte", colorfmt=self.colorfmt)

    def update_b_boxes(self, b_boxes, gt_visible=True):
        self.image.clear_widgets()

        if b_boxes is None:
            return
        if not gt_visible:
            return

        bb_copy = b_boxes.copy()
        texture_width = self.image.texture.width
        texture_height = self.image.texture.height
        image_width = self.image.norm_image_size[0]
        image_height = self.image.norm_image_size[1]

        x_img = self.image.center_x - image_width / 2
        y_img = self.image.center_y - image_height / 2

        w_ratio = image_width / texture_width
        h_ratio = image_height / texture_height
        for n, b in enumerate(bb_copy):
            for i in range(4):
                b[i] = dp(b[i])
            if self.flipHoriz:
                min_x = texture_width - b[3]
                max_x = texture_width - b[1]
                b[1] = min_x
                b[3] = max_x
            if self.flipVert:
                min_y = texture_height - b[2]
                max_y = texture_height - b[0]
                b[0] = min_y
                b[2] = max_y

            width = w_ratio * float(b[3] - b[1])
            height = h_ratio * float(b[2] - b[0])
            if width == 0 and height == 0:
                break

            x = x_img + w_ratio * float(b[1])
            y = y_img + h_ratio * (texture_height - float(b[2]))

            try:
                bb_color = self.cm.colors[b[4] % len(self.cm.colors)] + (1,)
                label = b[4]
                box_item = LabeledBoundingBox(id='box_{}'.format(n),
                                              bb_color=bb_color,
                                              x=x, y=y,
                                              width=width, height=height,
                                              label=label)
            except IndexError:
                box_item = BoundingBox(id='box_{}'.format(n),
                                       bb_color=self.cm.colors[0],
                                       x=x, y=y,
                                       width=width, height=height)
            self.image.add_widget(box_item)

    def get_frame(self, time_value, time_window):
        data_dict = {}
        for v in self.visualisers:
            data_dict[v.data_type] = {}
            data_dict[v.data_type] = v.get_frame(time_value, time_window, **self.settings[v.data_type])
        self.data.update(data_dict)
