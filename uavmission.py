# used many some of the inbuilt functions from  dronekit sitl 
# used functions are from this website
# https://dronekit-python.readthedocs.io/en/latest/guide/auto_mode.html
# https://dronekit-python.readthedocs.io/en/latest/automodule.html



import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import math
import serial

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()

   
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def download_mission(vehicle):
    
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    

def get_current_mission(vehicle):
    
    print("Downloading mission")
    download_mission(vehicle)
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1
        
        
    return n_WP, missionList
    

def add_last_waypoint_to_mission(                                       
        vehicle,            
        wp_Last_Latitude,   
        wp_Last_Longitude,  
        wp_Last_Altitude):  
   
    
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Save the vehicle commands to a list
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)

    # Modify the mission as needed. For example, here we change the
    wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    
    return (cmds.count)    

def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True

def get_location_metres(original_location, dNorth, dEast):
    
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def drop_payload(): # main code for droping the payload as well as the blinking the led using arduino 
    print("paylaod drop loaction reached")
    serialcomm = serial.Serial('/dev/ttyUSB0', 9600)
    serialcomm.timeout = 1
    o=2
    while o>0:
         i="on"
         if i=="done":
             print("work finished")
             break

         serialcomm.write(i.encode())

         time.sleep(0.5)

         print(serialcomm.readline().decode('ascii'))
         o=o-1

   

    serialcomm.close()
    t=1
    p=1.225
    c=0.049
    a=0.0000262
    l=(p*c*a)
    q=l/4
    
    vx=5
    ax=-(q)*((vx)**2)
    ay= 9.8 - (q)*((vx)**2)
    x=y=0
    while True:
         x1 = (vx)*(t) + 0.5*(ax)*((t)**2)
         y1 = (vx)*(t) + 0.5*(ay)*((t)**2)
         x=x+x1
         y=y+y1
         if int(y) in [18,19,20,21,22,23,24]:
             print("range of payload from drop location",x)
             break
         t=t+0.1    
    droplat= -35.362483
    droplong= 149.165125 
    way4lat=-35.362452
    way4long=149.166185
    z=(way4long-droplong)/(way4lat-droplat)
    w=math.atan(z)
    rlat=droplat+((x)*(math.cos(w)))
    rlong=droplong+((x)*(math.sin(w)))
    print("PAYLOAD RECIEVE LATITUDE COORDINATES",rlat)
    print("PAYLOAD RECIEVE LATITUDE COORDINATES",rlong)
    






     
     
       

        
        








  
#-- Setup the commanded flying speed
gnd_speed = 10 # [m/s]


 
#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')
#vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)



    
    
        #--- Wait until a valid mission has been uploaded
n_WP, missionList = get_current_mission(vehicle)
time.sleep(2)
if n_WP >0:
    print ("A valid mission has been uploaded: takeoff!")
            
            
    
       
        #-- Add a fake waypoint at the end of the mission
add_last_waypoint_to_mission(vehicle, vehicle.location.global_relative_frame.lat, 
                                       vehicle.location.global_relative_frame.lon, 
                                       vehicle.location.global_relative_frame.alt)
print("Home waypoint added to the mission")
time.sleep(1)
#-- Takeoff
arm_and_takeoff(20)
        
#-- Change the UAV mode to AUTO
print("Changing to AUTO")
ChangeMode(vehicle,"AUTO")
        
#-- Change mode, set the ground speed
vehicle.groundspeed = gnd_speed

mode = 'MISSION'
print ("Swiitch mode to MISSION")
        
while True:
        
      nextwaypoint=vehicle.commands.next
      print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
      
      if vehicle.commands.next == 4:
           drop_payload()
           
           
           print("Payload dropped")
           time.sleep(5)
      if vehicle.commands.next == vehicle.commands.count:
           print ("Final waypoint reached: go back home")
            #-- First we clear the flight mission
           clear_mission(vehicle)
           print ("Mission deleted")
            
            #-- We go back home
           
           break
            
print("Back to home")   
vehicle.mode = VehicleMode("RTL")    
    
    
time.sleep(0.5)
