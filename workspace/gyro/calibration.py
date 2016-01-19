#!/usr/bin/python

import smbus
import math
import time

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def millis():
  return int(round(time.time()*1000))

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

lastMillis = millis()
minX = 0.0
maxX = 0.0

def checkMinMaxOnX(actVal):
  global minX, maxX
  if actVal > maxX:
    maxX = actVal
    #print "New Max! Val:", maxX, "Diff:", maxX - minX
  if actVal < minX:
    minX = actVal
    #print "New Min! Val:", minX, "Diff:", maxX - minX

gyro_xout = 0.0

def readX():
  global gyro_xout
  gyro_xout = read_word_2c(0x47)

def calibrate():
  global lastMillis, gyro_xout, minX, maxX
  iterOffset = 30000
  i=0
  while(i<iterOffset):
    readX()
    i+=1
    time.sleep(0.01)
    if(i%1000==0):
      print "dump", i

  print "dumb vals ended"

  minX = gyro_xout
  maxX = gyro_xout
  avgX = 0.0
  i=0
  iterations = 10000
  while(i<iterations):
    readX()
    checkMinMaxOnX(gyro_xout)
    avgX += gyro_xout
    i+=1
    if(i%100==0):
      print i
    time.sleep(0.01)
  avgX /= iterations
  print minX, maxX, avgX
  return avgX

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

accumulateX = 0.0

print "gyro data"
print "---------"
avgX = calibrate()
while 1:
  readX()

  accel_x = read_word_2c(0x3b)
  accel_y = read_word_2c(0x3d)
  accel_z = read_word_2c(0x3f)

  gyro_xout = read_word_2c(0x43)
  gyro_yout = read_word_2c(0x45)
  gyro_zout = read_word_2c(0x47)

#  currMillis = millis()
#  dt = currMillis - lastMillis
#  lastMillis = currMillis

  angleX = ((gyro_xout - avgX)/131) * (dt/1000.0)
  #print angleX
  #angleX -= avgX
#  print angleX
#  if(angleX<-0.036 or (angleX>-0.004)):
  accumulateX += angleX
  #print accumulateX

  x_rot = get_x_rotation(accel_x, accel_y, accel_z)
  #print x_rot
  
  alpha = 0.96
  angle_x = alpha*accumulateX + (1.0 - 0.96) * x_rot
  print accumulateX,"\t\t\t",angle_x
  time.sleep(0.005)
