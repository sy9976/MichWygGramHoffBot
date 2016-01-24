#!/usr/bin/python
from thread import *
from bluetooth import *
from signal import *
import gps
import time
import RPi.GPIO as GPIO
import math
import smbus
from datetime import datetime
from datetime import timedelta
import sys
sys.path.append("/home/pi/Desktop/MichWygGramHoffBot/tests/mag_gyro_repo/raspi/i2c-sensors")
from bitify.python.utils.i2cutils import i2c_raspberry_pi_bus_number
import bitify.python.sensors.oldimu as imu


#--------- variables ----------
WHEELS_DIAMETER = 1 
WHEELS_PIN1 = 16
WHEELS_PIN2 = 18
WHEELS_FILE1 = '/home/pi/Desktop/MichWygGramHoffBot/log/wheels1_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'
WHEELS_FILE2 = '/home/pi/Desktop/MichWygGramHoffBot/log/wheels2_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'
POINTS_FILE = '/home/pi/Desktop/MichWygGramHoffBot/log/points_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'

PWM_PIN_M1 = 7 #11
DIR_PIN1_M1 = 21 # 10
DIR_PIN2_M1 = 19 # 9
#PWM_PIN_M2 = 
DIR_PIN1_M2 = 15 
DIR_PIN2_M2 = 13

PWM_FREQUENCY = 200
START_PWM = 75
PWM_STEP = 5

server_sock = None
lat = 0
lon = 0
cTime = "brak"
bus = None
compassError = False
sigterm_flag = False
connected = False
pointsOn = False
pwmValue = START_PWM
currentLevel = 0
calibration = False
x_off = 0
y_off = 0

#---------------------------GLOBAL VARIABLES-------------------------
last_read_time = 0.0
last_x_angle = 0.0#Filtered angles
last_y_angle = 0.0

mag_scale = 0.92
g_bearing = 0.0
unfiltered_bearing = 0.0
pitch = 0.0
roll = 0.0
yaw = 0.0

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

#ADDRS
GYRO_ADDR = 0x68
MAG_ADDR = 0x1e

def millis():
  return int(round(time.time()*1000))

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

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


def write_byte(address, adr, value):
    bus.write_byte_data(address, adr, value)

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
  global mag_scale
  global x_off
  global y_off
  mag_x = read_word_2c(MAG_ADDR, 3) * mag_scale - x_off
  mag_y = read_word_2c(MAG_ADDR, 7) * mag_scale - y_off
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
#----------------------------------------------

def gyro_thread():
  
  #--------------------------------------------------------------------------

  imu_controller = imu.OLDIMU(bus, 0x68, 0x1e, "OLDIMU")

  #while(1):
  #  (pitch, roll, yaw) = imu_controller.read_pitch_roll_yaw()
  #  result = "%.2f %.2f %.2f" % (pitch, roll, yaw)
  #  print result
  #--------------------------------------------------------------------------



  #bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
  global mag_scale
  #INIT MAG
  write_byte(MAG_ADDR, 0, 0b01110000)
  write_byte(MAG_ADDR, 1, 0b00100000)
  write_byte(MAG_ADDR, 2, 0b00000000)

  #INIT GYRO
  write_byte(GYRO_ADDR, power_mgmt_1, 0)
  setup()
  base_x_gyro = -436.9145
  base_y_gyro = -426.9437
  base_z_gyro = -277.1055

  accum_x = 0.0
  accum_y = 0.0
  accum_z = 0.0
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
    global g_bearing
    global unfiltered_bearing
    #(g_bearing, unfiltered_bearing) = read_compensated_angle(angle_x, angle_y)
    global x_off
    global y_off
    #[FIX ME]
    imu_controller.set_compass_offsets(x_off, y_off, 0)
    #(pitch, roll, yaw) = imu_controller.read_pitch_roll_yaw()
    global pitch
    global roll
    global yaw
    (pitch, roll, yaw) = imu_controller.read_pitch_roll_yaw()
    result = "%.2f %.2f %.2f" % (pitch, roll, yaw)
    
    print result
    #print g_bearing, unfiltered_bearing
    time.sleep(0.005)





#---------- compass ----------
#def read_byte(adr):
#    return bus.read_byte_data(address, adr)

#def read_word(adr):
#    high = bus.read_byte_data(address, adr)
#    low = bus.read_byte_data(address, adr+1)
#    val = (high << 8) + low
#    return val

#def read_word_2c(adr):
#    val = read_word(adr)
#    if (val >= 0x8000):
#        return -((65535 - val) + 1)
#    else:
#        return val

#def write_byte(adr, value):
#    bus.write_byte_data(address, adr, value)
#-------------------------------------------------------------


def points_thread(client_sock):
  x = 0
  y = 0
  while not sigterm_flag and connected:
    if pointsOn:
      client_sock.send("point " + str(x) + " " + str(y))
    x += 0.3
    y += 0.5
    time.sleep(1.5)
  print "points_thread end"
#---------- wheels ----------
def wheels_thread(pin, filename, client_sock):
  prevState = GPIO.input(pin)
  counter = 0
  lastX = 0
  lastY = 0
  scale = 0.92
  #while True:
  while not sigterm_flag and connected:
    actState = GPIO.input(pin)
    #if actState:
    #  print "1"
    #else:
    #  print "0"
    global x_off
    global y_off
    if ((not prevState) and (actState)):
      time.sleep(3000/1000000.0) #3ms
      actState = GPIO.input(pin)
      if (actState):
        counter += 1
        print "[" + str(pin) + "]Liczba zboczy: " + str(counter)
        fsock = open(filename, 'a')
        fsock.write(str(counter) + "\n")
        fsock.close()
        if(pin == WHEELS_PIN1): 
          x_out = read_word_2c(MAG_ADDR, 3) - x_off #- 89)* scale
          y_out = read_word_2c(MAG_ADDR, 7) - y_off#+ 436)* scale
          z_out = read_word_2c(MAG_ADDR, 5) * scale

          bearing  = math.atan2(y_out, x_out) 
          if (bearing < 0):
              bearing += 2 * math.pi
          
          x = math.sin(bearing) * WHEELS_DIAMETER + lastX
          y = math.cos(bearing) * WHEELS_DIAMETER + lastY
          print "Bearing: ", math.degrees(bearing)
          print "X: " + str(x) + " Y: " + str(y)
          if pointsOn:
            client_sock.send("point " + str(x) + " " + str(y))
          lastX = x
          lastY = y
    time.sleep(20000/1000000.0) #20ms
    prevState = actState

#---------- interrupt ----------
def sigint_handler(_signo, _stack_frame):
	print "test"
	global sigterm_flag
	sigterm_flag = True
	GPIO.cleanup()
	print "GPIO CLEANUP"
	server_sock.close()
	print "all done"

#---------- gps ----------
def gps_thread():
  #while True:
  while not sigterm_flag and connected:
    report = session.next()
    if report['class'] == 'TPV':
      if hasattr(report, 'cTime'):
        print "czas:",report.cTime
        global cTime
        cTime = report.cTime
      if hasattr(report, 'lat'):
        print "\tszerokosc:\t",report.lat
        global lat
        lat = report.lat
      if hasattr(report, 'lon'):
        print "\tdlugosc:\t",report.lon
        global lon
        lon = report.lon
      if hasattr(report, 'speed'):
        print "\tszybkosc:\t",report.speed
    if report['class'] == 'SKY':
      if hasattr(report, 'satellites'):
         print "\tsatelity:\t", len(report.satellites),
         satsUsed = 0
         for st in report.satellites:
           if hasattr(st, 'used'):
              if st.used==True:satsUsed+=1
         print "(uzywane: ", satsUsed, ")"
      if hasattr(report, 'hdop'):
         print "\thdop:\t\t", report.hdop
#--------------- magnetometer ----------------------#
def magnetometer_thread(client_sock):
  firstSample = True
  maxX = 0
  maxY = 0
  minX = 0
  minY = 0
  global calibration
  global x_off
  global y_off
  while(calibration):
    #------------------------------------------------
    #------------------------------------------------

    x = read_word_2c(MAG_ADDR, 3) * 0.92
    y = read_word_2c(MAG_ADDR, 7) * 0.92
    z = read_word_2c(MAG_ADDR, 5) * 0.92
    #print (str(x) + " " +  str(y) + " " + str(z))
  #  print(hmc5883l)
    
    #x -= 216.48
    #y -= 509.28
    #x = x * 1.12
    x_cal = x - x_off
    y_cal = y - y_off
    bearing  = math.atan2(y_cal, x_cal)
    if (bearing < 0):
        bearing += 2 * math.pi
    print("Bearing: "  + str(math.degrees(bearing)))
    if firstSample:
      minX = x_cal
      maxX = x_cal
      minY = y_cal
      maxY = y_cal
      firstSample = False
    else:
      minX = min(minX, x_cal)
      maxX = max(maxX, x_cal)
      minY = min(minY, y_cal)
      maxY = max(maxY, y_cal)
    #client_sock.send("point " + str(x_cal) + " " + str(y_cal))
    bearing2  = math.atan2(y_off, x_off)
    if (bearing2 < 0):
        bearing2 += 2 * math.pi

    #print("Bearing2: "  + str(math.degrees(bearing2)))
    time.sleep(0.5)
    fsock = open("/home/pi/Desktop/punkty.txt", 'a')
    fsock.write(str(x_cal) + " " +  str(y_cal) + " " + str(z) + "\n")
    fsock.close()
  client_sock.send("minX " + str(minX) + " maxX " + str(maxX) + " minY " + str(minY) + " maxY " + str(maxY))
  x_off += (maxX + minX)/2
  y_off += (maxY + minY)/2
  client_sock.send("x_off " + str(x_off) + " y_off " + str(y_off))
#---------- pwm ----------------

def change_pwm(motors, newLevel):
	#print "start change_pwm, newLevel: " + str(newLevel) + " currentLevel: " + str(currentLevel)
  dir = 0
  diff = abs(newLevel) - abs(currentLevel)
  if diff > 0:
	  dir = 1
  else:
    dir = -1
  diff = abs(diff)
  for i in range(diff):
    global pwmValue
    pwmValue += PWM_STEP * dir
    motors.ChangeDutyCycle(pwmValue)
    print "pwmValue " + str(pwmValue)
    time.sleep(0.1)
    print "change_pwm " + str(pwmValue)

def stop_pwm(motors):
	GPIO.output(DIR_PIN1_M1, False)#  GPIO.output(DIR_PIN2_M1, False)
	GPIO.output(DIR_PIN2_M1, False)
	motors.ChangeDutyCycle(START_PWM)
	global pwmValue
	pwmValue = START_PWM
	#print "STOP PWM"
	
def set_motors(mainMotors, newLevel, direction):
	global currentLevel
	print "START SET MOTORS newlvl: " + str(newLevel) + " currlvl " + str(currentLevel)
	if newLevel != currentLevel:
		dir = currentLevel * newLevel
		if newLevel == 0:	#stop
			stop_pwm(mainMotors)
		elif dir > 0:	#jedz w tym samym kierunku
			#print "dir > 0"
			change_pwm(mainMotors, newLevel)
		else: #jedz w przeciwnym kierunku #dir < 0 albo dir = 0 co oznacza, ze poprzednio byl stop
			#print "dir < 0 lub dir = 0"
			stop_pwm(mainMotors) #najpierw stop
			if newLevel > 0:
				GPIO.output(DIR_PIN1_M1, False)
				GPIO.output(DIR_PIN2_M1, True)
				currentLevel = 0 #aby roznica nie byla wieksza niz 5
				change_pwm(mainMotors, newLevel)
			else: #newLevel < 0
				GPIO.output(DIR_PIN1_M1, True)
				GPIO.output(DIR_PIN2_M1, False)
				currentLevel = 0 #aby roznica nie byla wieksza niz 5
				change_pwm(mainMotors, newLevel)
	currentLevel = newLevel
	
	if direction == "LEFT":
		GPIO.output(DIR_PIN1_M2, False)
		GPIO.output(DIR_PIN2_M2, True)
	elif direction == "RIGHT":
		GPIO.output(DIR_PIN1_M2, True)
		GPIO.output(DIR_PIN2_M2, False)
	else:
		GPIO.output(DIR_PIN1_M2, False)
		GPIO.output(DIR_PIN2_M2, False)
#---------- bluetooth ----------
def main_thread(client_sock, mainMotors):
  try:
    #while True:
    while not sigterm_flag:
      data = client_sock.recv(1024)
      splitedData = data.split()
      if data == "gps":
        gps_data = "gps " + str(lat) + " " + str(lon) + " " + cTime
        client_sock.send(gps_data)
        fsock = open(POINTS_FILE, 'a')
        fsock.write("\n" + str(lat) + " " + str(lon))
        fsock.close()
      elif splitedData[0] == "pwm":
        level = int(splitedData[1])
        direction = splitedData[2]
        set_motors(mainMotors, level, direction)
      elif data == "points on":
        global pointsOn
        pointsOn = True
        print "points on"
      elif data == "points off": 
        #global pointsOn
        pointsOn = False
        print "points off"
      elif data == "calib": 
        global calibration
        calibration = True
        start_new_thread(magnetometer_thread, (client_sock, ))
        print "calibration on"
      elif data == "talib": 
        #global calibration
        calibration = False
        print "calibration off"
      elif data == "dir": 
        print "dir"
        global g_bearing
        global unfiltered_bearing
        global pitch
        global roll
        global yaw
        #client_sock.send("dir" + str(g_bearing) + " " + str(unfiltered_bearing))
        client_sock.send("dir " + str(pitch) + " " + str(roll) + " " + str(yaw))
      elif data == "off":
        print "disconnected"
        client_sock.send("off ok")
        #client_sock.close()
        global connected
        connected = False
        break
      print "received [%s]" % data
  except IOError:
    pass
  #server_sock.close()

#main()

#--------- main ---------
print "START"
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(WHEELS_PIN1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(WHEELS_PIN2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(PWM_PIN_M1, GPIO.OUT)
GPIO.setup(DIR_PIN1_M1, GPIO.OUT)
GPIO.setup(DIR_PIN2_M1, GPIO.OUT)
#GPIO.setup(PWM_PIN_M2, GPIO.OUT)
GPIO.setup(DIR_PIN1_M2, GPIO.OUT)
GPIO.setup(DIR_PIN2_M2, GPIO.OUT)

GPIO.output(DIR_PIN1_M1, False)#  GPIO.output(DIR_PIN2_M1, False)
GPIO.output(DIR_PIN2_M1, False)

GPIO.output(DIR_PIN1_M2, False)#  GPIO.output(DIR_PIN2_M1, False)
GPIO.output(DIR_PIN2_M2, False)

mainMotors = GPIO.PWM(PWM_PIN_M1,PWM_FREQUENCY)
mainMotors.start(START_PWM)
#gps_configure()
#--------- compass ---------
try:
  bus = smbus.SMBus(1)
  #address = 0x1e
  write_byte(MAG_ADDR, 0, 0b01110000) # Set to 8 samples @ 15Hz
  write_byte(MAG_ADDR, 1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
  write_byte(MAG_ADDR, 2, 0b00000000) # Continuous sampling
except IOError:
  print "Blad! Sprawdz podlaczenie magnetometru!"
  compassError = True

#signal(SIGINT, sigint_handler)
start_new_thread(gps_thread, ())
start_new_thread(gyro_thread, ())
#while True:
while not sigterm_flag:
  server_sock=BluetoothSocket( RFCOMM )
  server_sock.bind(("",PORT_ANY))
  server_sock.listen(1)

  uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
  port = server_sock.getsockname()[1]

  advertise_service( server_sock, "SampleServer",
      service_id = uuid,
      service_classes = [ uuid, SERIAL_PORT_CLASS ],
      profiles = [ SERIAL_PORT_PROFILE ], 
      #protocols = [ OBEX_UUID ] 
      )

  print "Waiting for connection on RFCOMM channel %d" % port
  client_sock, client_info = server_sock.accept()
  print "Accepted connection from ", client_info
  start_new_thread(main_thread, (client_sock, mainMotors, ))
  connected = True
  if compassError:
    client_sock.send("Magnetometr jest odlaczony!")
    start_new_thread(points_thread, (client_sock, ))
  else:
    start_new_thread(wheels_thread, (WHEELS_PIN1, WHEELS_FILE1, client_sock, ))
    start_new_thread(wheels_thread, (WHEELS_PIN2, WHEELS_FILE2, client_sock, ))
    #start_new_thread(magnetometer_thread, (client_sock, ))
  while connected:
    time.sleep(1)
  print "off ok"
