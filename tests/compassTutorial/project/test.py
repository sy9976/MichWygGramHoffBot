from i2clibraries import i2c_hmc5883l
from time import sleep
import math

hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)

hmc5883l.setContinuousMode()
hmc5883l.setDeclination(0,0)

while(1):
  x, y, z = hmc5883l.getAxes()
  print (str(x) + " " +  str(y) + " " + str(z))
#  print(hmc5883l)
  bearing  = math.atan2(y, x)
  if (bearing < 0):
      bearing += 2 * math.pi
  #print("Bearing: "  + str(math.degrees(bearing)))
  x_off = x + 72.68
  y_off = y + 69
  bearing2  = math.atan2(y_off, x_off)
  if (bearing2 < 0):
      bearing2 += 2 * math.pi

  #print("Bearing2: "  + str(math.degrees(bearing2)))
  sleep(0.5)
