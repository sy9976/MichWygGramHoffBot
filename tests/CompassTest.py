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

write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
write_byte(2, 0b00000000) # Continuous sampling

scale = 1 #0.92

minx = 9999
miny = 9999
maxx = -9999
maxy = -9999
while True:
  x_out = read_word_2c(3) * scale
  y_out = read_word_2c(7) * scale
  z_out = read_word_2c(5) * scale
  if x_out < minx:
      minx = x_out
  if y_out < miny:
      miny = y_out
  if x_out > maxx:
      maxx = x_out
  if y_out > maxy:
      maxy = y_out
  bearing  = math.atan2(y_out, x_out) 
  if (bearing < 0):
      bearing += 2 * math.pi

  print "Bearing: ", math.degrees(bearing)
  print "X: " + str(x_out) + " Y: " + str(y_out) + " Z: " + str(z_out)
  print "x offset: ", (maxx + minx) / 2
  print "y offset: ", (maxy + miny) / 2
  print "x min: ", minx
  print "x max: ", maxx
  print "y min: ", miny
  print "y max: ", maxy

  sleep(1)
