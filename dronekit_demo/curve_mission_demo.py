# 控制小车曲线行驶的上位机控制程序

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
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
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,   #command
        0,   #confirmation
        0, 0, 0, 0,   #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)


def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to
    the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0  #Radius of "spherical" earth
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

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def adds_curve_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission.
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state
    (you must have called download at least once in the session and after clearing the mission)
    """

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear()

    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class.

    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                      0, 0, 0, 0, 0, 0,
                      0, 0, 10))
    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_offset_meters(aLocation, aSize, aSize,0)
    point2 = get_location_offset_meters(aLocation, 2 * aSize,0,0)
    point3 = get_location_offset_meters(aLocation, 3 * aSize,aSize,0)
    point4 = get_location_offset_meters(aLocation, 4 * aSize, 0,0)
    cmds.add(Command( 0, 0, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 0, 0, 0, 0, 0,
                      point1.lat, point1.lon, point1.alt))
    cmds.add(Command( 0, 0, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 0, 0, 0, 0, 0,
                      point2.lat, point2.lon, point2.alt))
    cmds.add(Command( 0, 0, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 0, 0, 0, 0, 0,
                      point3.lat, point3.lon, point3.alt))
    cmds.add(Command( 0, 0, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 0, 0, 0, 0, 0,
                      point4.lat, point4.lon, point4.alt))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 0, 0, 0, 0, 0,
                      point4.lat, point4.lon, point4.alt))

    print(" Upload new commands to vehicle")
    cmds.upload()


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
    adds_curve_mission(vehicle.location.global_frame, 50)
    print " Velocity: %s" % vehicle.groundspeed
    print "altitude: %s" % vehicle.location.global_frame.alt
    print("Starting mission")
    # Reset mission set to first (0) waypoint
    vehicle.commands.next = 0

    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")

    # Monitor mission.
    # Demonstrates getting and setting the command number
    # Uses distance_to_current_waypoint(), a convenience function for finding the
    #   distance to the next waypoint.

    while True:
        with open('log.txt', 'a') as f:
            f.write(time.asctime(time.localtime(time.time())) +
                        "lat: "+ str(vehicle.location.global_frame.lat) + " " +
                        "lon: " + str(vehicle.location.global_frame.lon) + " " +
                        "alt: " + str(vehicle.location.global_frame.alt) + " " +
                        "gps_fix: " + str(vehicle.gps_0.fix_type) + " " +
                        "gps_num_sat: " + str(vehicle.gps_0.satellites_visible) + " " +
                        "heading: " + str(vehicle.heading) +
                         '\n')
        nextwaypoint = vehicle.commands.next
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))

        # if nextwaypoint == 3:  # Skip to next waypoint
        #     print('Skipping to Waypoint 5 when reach waypoint 3')
        #     vehicle.commands.next = 4
        if nextwaypoint == 5:  # Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break
        time.sleep(1)

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
