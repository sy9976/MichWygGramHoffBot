#!/usr/bin/python
from thread import *
from bluetooth import *
from signal import *
import gps
import time
import RPi.GPIO as GPIO
import math
import smbus

WHEELS_DIAMETER = 1 

WHEELS_PIN1 = 16
WHEELS_PIN2 = 18

WHEELS_FILE1 = '/home/pi/Desktop/MichWygGramHoffBot/log/wheels1_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'
WHEELS_FILE2 = '/home/pi/Desktop/MichWygGramHoffBot/log/wheels2_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'
POINT_FILE = '/home/pi/Desktop/MichWygGramHoffBot/log/points_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'

server_sock = None
lat = 0
lon = 0
cTime = None

bus = None
compassError = False

session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(WHEELS_PIN1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(WHEELS_PIN2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


########################################## COMPASS
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

'''def compass_thread:
  while True:
    x_out = read_word_2c(3) * scale
    y_out = read_word_2c(7) * scale
    z_out = read_word_2c(5) * scale

    bearing  = math.atan2(y_out, x_out) 
    if (bearing < 0):
        bearing += 2 * math.pi

    print "Bearing: ", math.degrees(bearing)
    sleep(0.1)
'''
########################################    WHEELS
def wheels_thread(pin, filename, client_sock):
  prevState = GPIO.input(pin)
  counter = 0
  lastX = 0
  lastY = 0
  while True:
    actState = GPIO.input(pin)
    if (1):
      time.sleep(10000/10000.0) #300ms
      #actState = GPIO.input(pin)
      if (1):
        counter += 1
        print "[" + str(pin) + "]Liczba zboczy: " + str(counter)
        fsock = open(filename, 'a')
        fsock.write(str(counter) + "\n")
        fsock.close()
        if(pin == WHEELS_PIN1): 
          x_out = (read_word_2c(3) - 104)* scale #- 115.92
          y_out = (read_word_2c(7) + 700)* scale #+ 613.64
          z_out = read_word_2c(5) * scale

          bearing  = math.atan2(y_out, x_out) 
          if (bearing < 0):
              bearing += 2 * math.pi
          bearing = 0.0         
          x = math.sin(bearing) * WHEELS_DIAMETER + lastX
          y = math.cos(bearing) * WHEELS_DIAMETER + lastY
          print "Bearing: ", math.degrees(bearing)
          print "X: " + str(x) + " Y: " + str(y)
          client_sock.send("point " + str(x) + " " + str(y))
          lastX = x
          lastY = y
    #time.sleep(20000/1000000.0) #20ms
    prevState = actState

##########################################  INTERRUPT
def sigterm_handler(_signo, _stack_frame):
  print "test"
  server_sock.close()
  print "all done"

############################################    GPS
def gps_thread():
  while True:
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

#########################################BLUETOOTH
def main_thread(client_sock):
  try:
    while True:
      data = client_sock.recv(1024)
      if data == "gps":
        gps_data = "gps " + str(lat) + " " + str(lon) + " " + cTime
        client_sock.send(gps_data)
        fsock = open(POINTS_FILE, 'a')
        fsock.write("\n" + str(lat) + " " + str(lon))
        fsock.close()
      elif data == "start":
        break
        #gps
      elif data == "end": 
        break
      print "received [%s]" % data
  except IOError:
    pass

  print "disconnected"

  client_sock.close()
  #server_sock.close()

###########################################MAIN
print "START"

#####################################   COMPAS
try:
  bus = smbus.SMBus(1)
  address = 0x1e

  write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
  write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
  write_byte(2, 0b00000000) # Continuous sampling
except IOError:
  print "Blad! Sprawdz podlaczenie magnetometru!"
  compassError = True

scale = 0.92
####################################


signal(SIGTERM, sigterm_handler)
#start_new_thread(gps_thread, ())
while True:

  server_sock=BluetoothSocket( RFCOMM )
  server_sock.bind(("",PORT_ANY))
  server_sock.listen(1)

  uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
  port = server_sock.getsockname()[1]

  advertise_service( server_sock, "SampleServer",
      service_id = uuid,
      service_classes = [ uuid, SERIAL_PORT_CLASS ],
      profiles = [ SERIAL_PORT_PROFILE ], 
#                   protocols = [ OBEX_UUID ] 
      )

  print "Waiting for connection on RFCOMM channel %d" % port

  client_sock, client_info = server_sock.accept()
  print "Accepted connection from ", client_info
  start_new_thread(main_thread, (client_sock, ))
  if compassError:
    client_sock.send("Magnetometr jest odlaczony!")
  else:
    start_new_thread(wheels_thread, (WHEELS_PIN1, WHEELS_FILE1, client_sock, ))
   # start_new_thread(wheels_thread, (WHEELS_PIN2, WHEELS_FILE2, client_sock, ))

