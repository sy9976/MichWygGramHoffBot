#!/usr/bin/python
from thread import *
from bluetooth import *
from signal import *
import gps
import time
import RPi.GPIO as GPIO
import math
import smbus


#--------- variables ----------
WHEELS_DIAMETER = 1 
WHEELS_PIN1 = 16
WHEELS_PIN2 = 18
WHEELS_FILE1 = '/home/pi/Desktop/MichWygGramHoffBot/log/wheels1_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'
WHEELS_FILE2 = '/home/pi/Desktop/MichWygGramHoffBot/log/wheels2_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'
POINT_FILE = '/home/pi/Desktop/MichWygGramHoffBot/log/points_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'

PWM_PIN_M1 = 7 #11
DIR_PIN1_M1 = 19 # 10
DIR_PIN2_M1 = 21 # 9
#PWM_PIN_M2 = 
DIR_PIN1_M2 = 13 
DIR_PIN2_M2 = 15

PWM_FREQUENCY = 200
START_PWM = 75
PWM_STEP = 5

server_sock = None
lat = 0
lon = 0
cTime = None
bus = None
compassError = False
sigterm_flag = False
connected = False
pointsOn = False
pwmValue = START_PWM
currentLevel = 0

#---------- compass ----------
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
          x_out = (read_word_2c(3) - 89)* scale
          y_out = (read_word_2c(7) + 436)* scale
          z_out = read_word_2c(5) * scale

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
  while not sigterm_flag:
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
def magnetometer_thread():
  while(1):
    x = read_word_2c(3) * 0.96
    y = read_word_2c(7) * 0.96
    z = read_word_2c(5) * 0.96
    #print (str(x) + " " +  str(y) + " " + str(z))
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
    time.sleep(0.1)
    fsock = open("/home/pi/Desktop/punkty.txt", 'a')
    fsock.write(str(x) + " " +  str(y) + " " + str(z) + "\n")
    fsock.close()
  


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
        pointsOn = False
        print "points off"
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
  address = 0x1e
  write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
  write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
  write_byte(2, 0b00000000) # Continuous sampling
except IOError:
  print "Blad! Sprawdz podlaczenie magnetometru!"
  compassError = True

#signal(SIGINT, sigint_handler)
start_new_thread(gps_thread, ())
start_new_thread(magnetometer_thread, ())
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
  while connected:
    time.sleep(1)
  print "off ok"
