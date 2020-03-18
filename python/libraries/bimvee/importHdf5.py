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

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)

Using hickle to add hierarchical lists and dicts to hdf5 automatically
https://github.com/telegraphic/hickle
In fact, thess are just thin wrappers around hickle.dump/load, 
to offer a similar export function to other export calls.
pack/unpackWorkspaceVars are facilities that allows extra random data WIP 
to get bundled together. All files are included in both the import and export script for convenience. 
TODO: This script to import and pass through functions from exportHdf5
"""
#%%

# local imports
if __package__ is None or __package__ == '':
    from exportHdf5 import *
else:
    from .exportHdf5 import *
