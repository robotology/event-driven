# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

THis fallback module allows tqdm import to be optional; if tqdm is not installed 
then this module gets picked up in its place offering the minimal functions 
which allow calls to tqdm within the rest of the library to not fail.
"""

def tqdm(*args, **kwargs):
    print('You are using the local tqdm')
    if len(args) == 1:
        return args[0]
    else:
        return Pbar()

def trange(*args, **kwargs):
    print('You are using the local tqdm')
    if len(args) == 1:
        return args[0]
    else:
        return Pbar()

class Pbar():
    
    def update(*args):
        return None
