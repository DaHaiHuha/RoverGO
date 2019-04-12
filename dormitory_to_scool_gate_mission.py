from dronekit import connect, VehicleMode, Command,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import Tkinter
import Queue
import threading

point1 = LocationGlobal(36.359806, 120.6839358, 15.22)
point2 = LocationGlobal(36.3598868, 120.6818545, 15.08)
point3 = LocationGlobal(36.3598782, 120.6817293, 0.38)
point4 = LocationGlobal(36.3617882, 120.68172, 17.72)
point5 = LocationGlobal(36.3618656, 120.6797568, 2.12)
point6 = LocationGlobal(36.3652845, 120.6802798, 16.88)
point7 = LocationGlobal(36.3658947, 120.6802497, 16.71)
point8 = point7

#STOP_FLAG = 0
vehicle = connect('/dev/ttyACM0', wait_ready=True)
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

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def adds_mission():
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
                      0, 0, 15))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, point1.alt))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, point2.alt))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, point3.alt))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, point4.alt))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  0, 0, 0, 0, 0, 0, point5.lat, point5.lon, point5.alt))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  0, 0, 0, 0, 0, 0, point6.lat, point6.lon, point6.alt))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  0, 0, 0, 0, 0, 0, point7.lat, point7.lon, point7.alt))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  0, 0, 0, 0, 0, 0, point8.lat, point8.lon, point8.alt))
    #add dummy waypoint at final waypoint (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point8.lat, point8.lon, point8.alt))

    print(" Upload new commands to vehicle")
    cmds.upload()

def set_roi():
    # create the MAV_CMD_DO_SET_ROI command
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        targetWaypointLocation.lat,
        targetWaypointLocation.lon,
        targetWaypointLocation.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)


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


def run():
    arm_vehicle()
    adds_mission()
    vehicle.groundspeed = 0.1
    print " Velocity: %s" % vehicle.groundspeed
    print "altitude: %s" % vehicle.location.global_frame.alt
    print("Starting mission")
    # Reset mission set to first (0) waypoint
    vehicle.commands.next = 0

    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")
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

        nextwaypoint = vehicle.commands.next
        time.sleep(1)
        set_roi()
        time.sleep(2)
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
        #print(STOP_FLAG)
        if nextwaypoint == 8 :  # Dummy waypoint - as soon as we reach final waypoint this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint")
            break
        #time.sleep(1)

    print "altitude: %s" % vehicle.location.global_frame.alt
    time.sleep(10)
    print "state %s" % vehicle.mode
    vehicle.mode = VehicleMode('HOLD')
    print "state %s" % vehicle.mode
    vehicle.close()

def stop():
    if vehicle.armed == False or vehicle.mode == "HOLD" :
        print "The vehicle has already stopped!"
    else:
        vehicle.mode = VehicleMode('HOLD')
        #STOP_FLAG = 1
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

