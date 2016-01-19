#!/usr/bin/python

import smbus
import math
import time
from datetime import datetime
from datetime import timedelta

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

'''def millis():
  dt = datetime.now()
  ms = (dt.day * 24 * 60 * 60 + dt.second) * 1000 + dt.microsecond / 1000.0
  return ms
'''

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

#---------------------------GLOBAL VARIABLES-------------------------
last_read_time = 0.0

last_x_angle = 0.0#Filtered angles
last_y_angle = 0.0
last_z_angle = 0.0

last_gyro_x_angle = 0.0#Stored gyro angles
last_gyro_y_angle = 0.0
last_gyro_z_angle = 0.0

base_x_accel = 0.0 #calibrated values
base_y_accel = 0.0
base_z_accel = 0.0

base_x_gyro = 0.0 #calibrated values
base_y_gyro = 0.0
base_z_gyro = 0.0

#----------------------------SETTERS----------------------------------

def set_last_time(time):
  global last_read_time 
  last_read_time= time

def set_last_angles(x, y, z):
  global last_x_angle
  last_x_angle = x
  global last_y_angle
  last_y_angle = y
  global last_z_angle
  last_z_angle = z

def set_last_gyro_angles(x, y, z):
  global last_gyro_x_angle
  last_gyro_x_angle = x
  global last_gyro_y_angle
  last_gyro_y_angle = y
  global last_gyro_z_angle
  last_gyro_z_angle = z

#----------------------------CALIBRATION----------------------------------
def calibrate_sensors():
  num_readings = 10
  x_accel = 0.0
  y_accel = 0.0
  z_accel = 0.0
  x_gyro = 0.0
  y_gyro = 0.0
  z_gyro = 0.0

  dumbVar = 0.0
  #DISCARD FIRST READ
  dumbVar = read_word_2c(0x43)
  dumbVar = read_word_2c(0x45)
  dumbVar = read_word_2c(0x47)
                               
  dumbVar = read_word_2c(0x3b)
  dumbVar = read_word_2c(0x3d)
  dumbVar = read_word_2c(0x3f)

  i = 0
  while i < num_readings:
    x_gyro += read_word_2c(0x43)
    y_gyro += read_word_2c(0x45)
    z_gyro += read_word_2c(0x47)

    x_accel += read_word_2c(0x3b)
    y_accel += read_word_2c(0x3d)
    z_accel += read_word_2c(0x3f)

    i += 1
    time.sleep(0.1)

  global base_x_accel 
  base_x_accel = x_accel / num_readings
  global base_y_accel 
  base_y_accel = y_accel / num_readings
  global base_z_accel 
  base_z_accel = z_accel / num_readings

  global base_x_gyro
  base_x_gyro = x_gyro / num_readings
  global base_y_gyro 
  base_y_gyro = y_gyro / num_readings
  global base_z_gyro 
  base_z_gyro = z_gyro / num_readings

#END CALIBRATION

def setup():
  #jakies writy
  #calibrate_sensors()
  set_last_time(millis())
  set_last_angles(0.0, 0.0, 0.0)
  set_last_gyro_angles(0.0, 0.0, 0.0)
  

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)
setup()
base_x_gyro = -436.9145
base_y_gyro = -426.9437
base_z_gyro = -277.1055

accum_x = 0.0
accum_y = 0.0
accum_z = 0.0
while 1:
  gyro_xout = read_word_2c(0x43)
  gyro_yout = read_word_2c(0x45)
  gyro_zout = read_word_2c(0x47)

  accel_x = read_word_2c(0x3b)
  accel_y = read_word_2c(0x3d)
  accel_z = read_word_2c(0x3f)

  t_now = millis()
  FS_SEL = 131.0
  
  gyro_x = (gyro_xout - base_x_gyro)/FS_SEL
  gyro_y = (gyro_yout - base_y_gyro)/FS_SEL
  gyro_z = (gyro_zout - base_z_gyro)/FS_SEL
  
  #Get angle values fom accelerometer
  RADIANS_TO_DEGREES = 180/3.14159
  #accel_angle_y = math.atan((-1)*accel_x/math.sqrt(accel_y*accel_y + accel_z*accel_z))*RADIANS_TO_DEGREES
  #accel_angle_x = math.atan(accel_y/math.sqrt(accel_x*accel_x + accel_z*accel_z))*RADIANS_TO_DEGREES
  accel_angle_x = get_x_rotation(accel_x, accel_y, accel_z)
  accel_angle_y = get_y_rotation(accel_x, accel_y, accel_z)
  accel_angle_z = math.atan(math.sqrt(accel_x*accel_x + accel_y*accel_y)/accel_z)*RADIANS_TO_DEGREES

#  if(accel_angle_y < 0):
#    accel_angle_y += 360

#  if(accel_angle_x < 0):
#    accel_angle_x += 360

  #Compute the (filtered) gyro angles
  dt = (t_now - last_read_time)/1000.0

  tx = gyro_x*dt
  ty = gyro_y*dt
  tz = gyro_z*dt

  if(tx<0.009 and tx>-0.009):
    tx = 0.0
  if(ty<0.009 and ty>-0.009):
    ty = 0.0        
  if(tz<0.009 and tz>-0.009):
    tz = 0.0          
                              
  gyro_angle_x = tx + last_x_angle
  gyro_angle_y = ty + last_y_angle 
  gyro_angle_z = tz + last_z_angle
    
  #print gyro_x*dt, gyro_y*dt, gyro_z*dt
#  print accum_x, accum_y, accum_z
  #Compute the drifting gyro angles
  unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_x_angle
  unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_y_angle
  unfiltered_gyro_angle_z = gyro_z*dt + last_gyro_z_angle

  #Apply the complementary filter ...
  alpha = 0.96
  angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x
  angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y
  angle_z = alpha*gyro_angle_z + (1.0 - alpha)*accel_angle_z

  set_last_time(t_now)
  set_last_angles(angle_x, angle_y, angle_z)
  set_last_gyro_angles(unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z)
  
  #print "X\tY\tZ"
  print str(angle_x)+"\t"+  str(angle_y)+"\t" +str(angle_z)

  time.sleep(0.005)
