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
from kivy.properties import BooleanProperty, ObjectProperty, StringProperty
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
    data = ObjectProperty(force_dispatch=True)
    need_init = BooleanProperty(True)
    dsm = ObjectProperty(None, allownone=True)
    flipHoriz = BooleanProperty(False)
    flipVert = BooleanProperty(False)
    colorfmt = 'luminance'

    def on_dsm(self, instance, value):
        if self.dsm is not None:
            self.colorfmt = self.dsm.get_colorfmt()
            x, y = self.dsm.get_dims()
            buf_shape = (dp(x), dp(y))
            self.image.texture = Texture.create(size=buf_shape, colorfmt=self.colorfmt)
            self.need_init = False

    def __init__(self, **kwargs):
        super(Viewer, self).__init__(**kwargs)
        # self.image = Image()
        # self.add_widget(self.image)
        from matplotlib.pyplot import get_cmap
        self.cm = get_cmap('tab20')
        self.image.texture = None
        Clock.schedule_once(self.init, 0)

    def init(self, dt):
        self.data = np.zeros((1, 1), dtype=np.uint8)

    def on_data(self, instance, value):
        self.update_data(self.data)

    def update(self):
        self.update_data(self.data)

    def update_data(self, data):
        if self.need_init:
            self.on_dsm(None, None)
        if self.image.texture is not None:
            x, y = self.dsm.get_dims()
            size_required = x * y * (1 + (self.colorfmt == 'rgb') * 2)
            if not isinstance(data, np.ndarray):
                data = np.zeros((x, y, 3), dtype=np.uint8)
            if data.size >= size_required:
                try:
                    if self.flipHoriz:
                        data = np.flip(data, axis=1)
                    if not self.flipVert:  # Not, because by default, y should increase downwards, following https://arxiv.org/pdf/1610.08336.pdf
                        data = np.flip(data, axis=0)
                except AttributeError:
                    pass  # It's not a class that allows flipping
                self.image.texture.blit_buffer(data.tostring(), bufferfmt="ubyte", colorfmt=self.colorfmt)

    def get_frame(self, time_value, time_window):
        pass


class LabelableViewer(Viewer):
    b_boxes = ObjectProperty(force_dispatch=True)
    b_boxes_visible = BooleanProperty(False)

    def get_b_boxes(self, time_value):
        self.b_boxes = self.dsm.get_b_box(time_value)

    def update(self):
        self.update_data(self.data)
        self.update_b_boxes(self.b_boxes, self.b_boxes_visible)

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

    def on_b_boxes(self, instance, value):
        self.update_b_boxes(self.b_boxes, self.b_boxes_visible)


class ViewerDvs(Viewer):
    def __init__(self, **kwargs):
        super(ViewerDvs, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = imread('graphics/missing.jpg')
        else:
            kwargs = {
                'polarised': self.polarised,
                'contrast': self.contrast,
                'pol_to_show': self.pol_to_show
            }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)


class ViewerFrame(Viewer):
    def __init__(self, **kwargs):
        super(ViewerFrame, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = imread('graphics/missing.jpg')
        else:
            kwargs = {
            }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)


class LabelableViewerDvs(LabelableViewer):
    def __init__(self, **kwargs):
        super(LabelableViewerDvs, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = imread('graphics/missing.jpg')
        else:
            kwargs = {
                'polarised': self.polarised,
                'contrast': self.contrast,
                'pol_to_show': self.pol_to_show
            }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)
        self.get_b_boxes(time_value)


class LabelableViewerFrame(LabelableViewer):
    def __init__(self, **kwargs):
        super(LabelableViewerFrame, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = imread('graphics/missing.jpg')
        else:
            kwargs = {
            }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)
        self.get_b_boxes(time_value)


class ViewerPose6q(Viewer):
    def __init__(self, **kwargs):
        super(ViewerPose6q, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = imread('graphics/missing.jpg')
        else:
            kwargs = {
                'interpolate': self.interpolate,
                'perspective': self.perspective,
                    }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)


class ViewerPoint3(Viewer):
    def __init__(self, **kwargs):
        super(ViewerPoint3, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = imread('graphics/missing.jpg')
        else:
            kwargs = {
                'perspective': self.perspective,
                'yaw': self.yaw,
                'pitch': self.pitch,
                    }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)