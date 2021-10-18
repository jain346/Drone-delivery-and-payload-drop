# this code built the connection between the mission file with the arduino file using the python library pyserial
# this code exists in the mission.py 
# just to highlight it I commit it seperately



import serial
import time

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
