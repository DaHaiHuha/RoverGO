#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@Time : 2019-04-24 14:21
@Author : Cheng Alen
@Site : 
@File : auxfun.py
@Software: PyCharm
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


def print_sys(vehicle):
    print('Mode: ', vehicle.mode.name)
    print('vehicle.armed:  ', vehicle.armed)
    print("Groundspeed: %s" % vehicle.groundspeed)
    pass


def get_current_loc(vehicle, lat_change, lon_change):
    lat = vehicle.location.global_frame.lat + lat_change
    lon = vehicle.location.global_frame.lon + lon_change
    return LocationGlobalRelative(lat, lon, vehicle.location.global_frame.alt)


def parameter_show_and_change(vehicle):
    print("\nPrint all parameters (iterate `vehicle.parameters`):")
    for key, value in vehicle.parameters.iteritems():
        print(" Key:%s Value:%s" % (key, value))
    pass


def change_v(vehicle, forward=0, turn=0):
    """
    Use channel override to control the speed and turn rate
    Warning: Must in the Manual Mode
    :param forward: the range is (-100,100)
    :param turn: no range, normal choice is between (-10, 10)
    :return:
    """
    if vehicle.mode.name != 'MANUAL':
        vehicle.channels.overrides = {}
        vehicle.mode = VehicleMode('MANUAL')
    forward_speed = 1500 + forward
    turn_rate = 1500 + turn
    vehicle.channels.overrides['1'] = turn_rate
    vehicle.channels.overrides['3'] = forward_speed
    return None


def delete_channel_override(vehicle):
    vehicle.channels.overrides = {}


def change_turnrate(vehicle, turn=0):
    turn_rate = 1500 + turn
    vehicle.channels.overrides['1'] = turn_rate


def change_speed(vehicle, forward=0):
    forward_speed = 1500 + forward
    vehicle.channels.overrides['3'] = forward_speed
