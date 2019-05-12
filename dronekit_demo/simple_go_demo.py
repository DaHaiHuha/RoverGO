# 往北直行100米的上位机控制程序

from dronekit import connect, VehicleMode, LocationGlobalRelative,Command,LocationGlobal
from pymavlink import mavutil
import os
import time
import math
import Tkinter
import Queue
import threading

vehicle = connect('127.0.0.1:14551', wait_ready=True)
# vehicle = connect('127.0.0.1:14551', wait_ready=True)

def arm_vehicle():
    """
    Arms vehicle
    """
    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)
        break

    print "Give up waiting"

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

def set_roi(location):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


#______________________________________________________________________________________________

# Load commands
cmds = vehicle.commands
cmds.clear()

def run():
    arm_vehicle()
    # point1 = LocationGlobal(vehicle.location.global_frame.lat+0.01,
    #                         vehicle.location.global_frame.lon,
    #                         vehicle.location.global_frame.alt)

    vehicle.groundspeed = 1
    print " Velocity: %s" % vehicle.groundspeed
    print "altitude: %s" % vehicle.location.global_frame.alt

    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, 100, 0, 0)

    i = 0


    while (True):
        with open('log.txt', 'a') as f:
            f.write(time.asctime(time.localtime(time.time())) +
                        " lat: "+ str(vehicle.location.global_frame.lat) + " " +
                        "lon: " + str(vehicle.location.global_frame.lon) + " " +
                        "alt: " + str(vehicle.location.global_frame.alt) + " " +
                        "gps_fix: " + str(vehicle.gps_0.fix_type) + " " +
                        "gps_num_sat: " + str(vehicle.gps_0.satellites_visible) + " " +
                        "heading: " + str(vehicle.heading) +
                         '\n')
        i += 1
        # cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0,
        #               0, 0, 0, wp.lat, wp.lon, wp.alt)
        # cmds.add(cmd)
        # cmds.upload()
        vehicle.simple_goto(wp)
        print "I am running towards the waypoint!"
        time.sleep(1)
        set_roi(wp)
        print "I am adjusting my position!"
        time.sleep(1)
        #print "state %s" % vehicle.mode
        if get_distance_metres(home, wp) < 1 or i >= 60:
            break

    print "altitude: %s" % vehicle.location.global_frame.alt
    time.sleep(10)
    print "state %s" % vehicle.mode
    vehicle.mode = VehicleMode('HOLD')
    print "state %s" % vehicle.mode
    vehicle.close()

def stop():
    if vehicle.armed == False or vehicle.mode == "HOLD":
        print "The vehicle has already stopped!"
    else:
        vehicle.mode = VehicleMode('HOLD')
        print "Stop the vehicle successfully!"
        vehicle.armed = False
        vehicle.close()

class GuiPart():
    def __init__(self,master,queue,endCommand):
        self.queue=queue
        Tkinter.Button(master,text='Done',command=endCommand).pack()
    def processIncoming(self):
        while self.queue.qsize():
            try:
                msg=self.queue.get(0)
                print msg
            except Queue.Empty:
                pass
class ThreadedClient():
    def __init__(self,master):
        self.master=master
        self.queue=Queue.Queue()
        self.gui=GuiPart(master,self.queue,self.endApplication)
        self.running=True
        self.thread1=threading.Thread(target=self.workerThread1)
        self.thread1.start()
        self.periodicCall()
    def periodicCall(self):
        self.master.after(200,self.periodicCall)
        self.gui.processIncoming()
        if not self.running:
            self.master.destroy()
    def workerThread1(self):
        #self.ott=Tkinter.Tk()
        #self.ott.mainloop()
        while self.running:
            run()
    def endApplication(self):
        self.running=False
        stop()

root=Tkinter.Tk()
client=ThreadedClient(root)
root.mainloop()














