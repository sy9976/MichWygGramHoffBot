import smbus
import time
import math
from time import sleep

bus = smbus.SMBus(1)
address = 0x1e


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

def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)

#write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
#write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
#write_byte(2, 0b00000000) # Continuous sampling

#scale = 0.92
pointsfile = open('compass-plot3.dat', 'r')
#pointsfile = open('zkamilPoints.txt', 'r')
x_offset = 0.0
y_offset = 0.0
i = 0
while True:
#for i in range(0,500):
#  x_out = read_word_2c(3) * scale
#  y_out = read_word_2c(7) * scale
#  z_out = read_word_2c(5) * scale
  pointString = pointsfile.readline();
  stringsArray = pointString.split(' ')

  x_out = float(stringsArray[0])
  y_out = float(stringsArray[1])
  print str(x_out) + ' ' + str(y_out)
  if i==0:
    x_offset = x_out
    y_offset = y_out
    i=1
    continue
  sleep(0.1)
  bearing  = math.atan2(x_out - x_offset, y_out - y_offset) 
  if (bearing < 0):
      bearing += 2 * math.pi
  print 'Bearing: ' + str(math.degrees(bearing))
  x_offset = x_out
  y_offset = y_out
#  print x_out, y_out
  #print "Bearing: ", math.degrees(bearing)
