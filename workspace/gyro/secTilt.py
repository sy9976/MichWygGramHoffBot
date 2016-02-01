#!/usr/bin/python

import smbus
import math
import time
from datetime import datetime
from datetime import timedelta

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

#ADDRS
GYRO_ADDR = 0x68
MAG_ADDR = 0x1e

def millis():
  return int(round(time.time()*1000))

def read_byte(address, adr):
    return bus.read_byte_data(address, adr)

def read_word(address, adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(address, adr):
    val = read_word(address, adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def write_byte(address, adr, value):
    bus.write_byte_data(address, adr, value)

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

#---------------------------GLOBAL VARIABLES-------------------------
last_read_time = 0.0

last_x_angle = 0.0#Filtered angles
last_y_angle = 0.0

#----------------------------SETTERS----------------------------------

def set_last_time(time):
  global last_read_time 
  last_read_time= time

def set_last_angles(x, y):
  global last_x_angle
  last_x_angle = x
  global last_y_angle
  last_y_angle = y

def setup():
  set_last_time(millis())
  set_last_angles(0.0, 0.0)
  
#---------------------------MAGNET--------------------------------

def read_compensated_angle(pitch, roll):
  mag_x = read_word_2c(MAG_ADDR, 3) * mag_scale
  mag_y = read_word_2c(MAG_ADDR, 7) * mag_scale
  mag_z = read_word_2c(MAG_ADDR, 5) * mag_scale

  cos_pitch = math.cos(pitch)
  sin_pitch = math.sin(pitch)
  
  cos_roll = math.cos(roll)
  sin_roll = math.sin(roll)

  Xh = (mag_x * cos_roll) + (mag_z * sin_roll)
  Yh = (mag_x * sin_pitch * sin_roll) + (mag_y * cos_pitch) - (mag_z * sin_pitch * cos_roll)

  bearing = math.atan2(Yh, Xh)
  if(bearing < 0):
    return (bearing + (2 * math.pi), math.atan2(mag_y, mag_x) + (2*math.pi))
  else:
    return (bearing, math.atan2(mag_y, mag_x))

#----------------------------TEST2--------------------
def getMagAccHeading():
  cx = read_word_2c(MAG_ADDR, 3) * mag_scale
  cy = read_word_2c(MAG_ADDR, 7) * mag_scale
  cz = read_word_2c(MAG_ADDR, 5) * mag_scale

  ax = read_word_2c(GYRO_ADDR, 0x3b)
  ay = read_word_2c(GYRO_ADDR, 0x3d)
  az = read_word_2c(GYRO_ADDR, 0x3f)

  ayf = get_x_rotation(ax, ay, az)
  axf = get_y_rotation(ax, ay, az)

  xh = cx*math.cos(ayf)+cy*math.sin(ayf)*math.sin(axf)-cz*math.cos(axf)*math.sin(ayf)
  yh = cy*math.cos(axf)+cz*math.sin(axf)
  
  var_compass=math.atan2(yh, xh) * (180 / math.pi) -90
  if var_compass>0:
    var_compass-=360
  var_compass += 360

  return var_compass
bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards

#INIT MAG
write_byte(MAG_ADDR, 0, 0b01110000)
write_byte(MAG_ADDR, 1, 0b00100000)
write_byte(MAG_ADDR, 2, 0b00000000)

mag_scale = 0.92

#INIT GYRO
write_byte(GYRO_ADDR, power_mgmt_1, 0)
setup()
base_x_gyro = -436.9145
base_y_gyro = -426.9437
base_z_gyro = -277.1055

accum_x = 0.0
accum_y = 0.0
accum_z = 0.0
#-------------------second tut while
while 1:
  temp = getMagAccHeading()
  print temp
  time.sleep(0.005)


while 1:
  gyro_xout = read_word_2c(GYRO_ADDR, 0x43)
  gyro_yout = read_word_2c(GYRO_ADDR, 0x45)
  gyro_zout = read_word_2c(GYRO_ADDR, 0x47)

  accel_x = read_word_2c(GYRO_ADDR, 0x3b)
  accel_y = read_word_2c(GYRO_ADDR, 0x3d)
  accel_z = read_word_2c(GYRO_ADDR, 0x3f)

  t_now = millis()
  FS_SEL = 131.0
  
  gyro_x = (gyro_xout - base_x_gyro)/FS_SEL
  gyro_y = (gyro_yout - base_y_gyro)/FS_SEL
  gyro_z = (gyro_zout - base_z_gyro)/FS_SEL
  
  #Get angle values fom accelerometer
  RADIANS_TO_DEGREES = 180/3.14159
  accel_angle_x = get_x_rotation(accel_x, accel_y, accel_z)
  accel_angle_y = get_y_rotation(accel_x, accel_y, accel_z)

  #Compute the (filtered) gyro angles
  dt = (t_now - last_read_time)/1000.0

  tx = gyro_x*dt
  ty = gyro_y*dt

  if(tx<0.009 and tx>-0.009):
    tx = 0.0
  if(ty<0.009 and ty>-0.009):
    ty = 0.0        
                              
  gyro_angle_x = tx + last_x_angle
  gyro_angle_y = ty + last_y_angle 
    
  alpha = 0.96
  angle_x = alpha*tx + (1.0 - alpha)*accel_angle_x
  angle_y = alpha*ty + (1.0 - alpha)*accel_angle_y

  set_last_time(t_now)
  set_last_angles(angle_x, angle_y)
  
  #print "X\tY\tZ"
#  print str(angle_x)+"\t"+  str(angle_y)

  (bearing, unfiltered_bearing) = read_compensated_angle(angle_x, angle_y)
  print bearing, unfiltered_bearing
  time.sleep(0.005)
