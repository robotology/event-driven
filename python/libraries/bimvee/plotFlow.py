#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Sim Bamford
         Aiko Dinale
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
plotImu takes 'inDict' - a dictionary containing imported IMU data 
(or a higher level container, in which attempts to descend and call itself) 
as created by importAe, and plots against time the various dimensions of the 
IMU samples contained. 
"""
import os
import matplotlib.pyplot as plt
import seaborn as sns

sns.set(palette="colorblind")

# Color Palette for Color Blindness
zesty_palette     = ['#F5793A', '#A95AA1', '#85C0F9', '#0F2080']
retro_palette     = ['#601A4A', '#EE442F', '#63ACBE', '#F9F4EC']
corporate_palette = ['#8DB8AD', '#EBE7E0', '#C6D4E1', '#44749D']

#-----------------------------------------------------------------------------------------------------
def plotFlow(flowDict, fig_path=None, fig_name=None, fig_subtitle=None):
    """
    Plot the FLOW events in flowDict against the time. If specified, save the
    generated figure as fig_name.png at the location defined by fig_path.

    Arguments:
        flowDict {dict} -- dictionary of FLOW events as formatted by bimvee from event-driven library

    Keyword Arguments:
        fig_path {string} -- save path for the generated figure (default: {None})
        fig_name {string} -- name of the generated figure (default: {None})
        fig_subtitle {string} -- figure sub-title (default: {None})
    """
    fig = plt.figure(figsize=(16.0, 10.0))
    if isinstance(fig_subtitle, str):
        fig.suptitle("FLOW Events\n" + fig_subtitle, fontsize=20, fontweight='bold')
    else:
        fig.suptitle("FLOW Events", fontsize=20, fontweight='bold')


    ax11 = plt.subplot(2, 1, 1)
    plt.plot(flowDict['ts'], flowDict['vx'], color=retro_palette[0], marker='.', linewidth=0.0)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('Vx [px/s]', fontsize=10, fontweight='bold')
    plt.grid(True)

    plt.subplot(2, 1, 2, sharex=ax11)
    plt.plot(flowDict['ts'], flowDict['vy'], color=retro_palette[1], marker='.', linewidth=0.0)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('Vy [px/s]', fontsize=10, fontweight='bold')
    plt.grid(True)

    fig.tight_layout()
    if isinstance(fig_subtitle, str):
        fig.subplots_adjust(top=0.9)
    else:
        fig.subplots_adjust(top=0.95)

    fig.align_ylabels()

    if isinstance(fig_path, str) and isinstance(fig_name, str):
        plt.savefig(os.path.join(fig_path, fig_name + ".png"), dpi=300, bbox_inches='tight')
        print("Saving " + fig_name + ".png")
        plt.close()
    else:
        plt.show()

#-----------------------------------------------------------------------------------------------------
def plotFlowDistribution(flowDict, fig_path=None, fig_name=None, fig_subtitle=None):
    """
    Plot the distribution of the FLOW events in flowDict and save the generated
    figure as fig_name.png at the location defined by fig_path.

    Arguments:
        flowDict {dict} -- dictionary of FLOW events as formatted by bimvee from event-driven library

    Keyword Arguments:
        fig_path {string} -- save path for the generated figure (default: {None})
        fig_name {string} -- name of the generated figure (default: {None})
        fig_subtitle {string} -- figure sub-title (default: {None})
    """
    fig = plt.figure(figsize=(14.0, 10.0))
    if isinstance(fig_subtitle, str):
        fig.suptitle("FLOW Events Distribution\n" + fig_subtitle, fontsize=20, fontweight='bold')
    else:
        fig.suptitle("FLOW Events Distribution", fontsize=20, fontweight='bold')
    
    plt.subplot(2,1,1)
    sns.distplot(flowDict['vx'], bins=100, color = retro_palette[0])
    plt.xlabel('Vx [px/s]', fontsize=10, fontweight='bold')

    plt.subplot(2,1,2)
    sns.distplot(flowDict['vy'], bins=100, color = retro_palette[1])
    plt.xlabel('Vy [px/s]', fontsize=10, fontweight='bold')

    fig.tight_layout()
    if isinstance(fig_subtitle, str):
        fig.subplots_adjust(top=0.9)
    else:
        fig.subplots_adjust(top=0.95)

    if isinstance(fig_path, str) and isinstance(fig_name, str):
        plt.savefig(os.path.join(fig_path, fig_name + '.png'), dpi=300, bbox_inches='tight')
        print("Saving " + fig_name + ".png")
        plt.close()
    else:
        plt.show()
