#!/usr/bin/python
from thread import *
from bluetooth import *
from signal import *
import gps
import time
import RPi.GPIO as GPIO

WHEELS_PIN1 = 16
WHEELS_PIN2 = 18

WHEELS_FILE1 = '/home/pi/Desktop/MichWygGramHoffBot/log/wheels1_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'
WHEELS_FILE2 = '/home/pi/Desktop/MichWygGramHoffBot/log/wheels2_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'
POINT_FILE = '/home/pi/Desktop/MichWygGramHoffBot/src/points_' + time.strftime("%Y-%m-%d %H:%M") + '.txt'

server_sock = None
lat = 0
lon = 0
cTime = None

session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(WHEELS_PIN1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(WHEELS_PIN2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def wheels_thread(pin, filename):
  prevState = GPIO.input(pin)
  counter = 0
  while True:
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
        print "[1]Liczba zboczy: " + str(counter)
        fsock = open(filename, 'a')
        fsock.write(str(counter) + "\n")
        fsock.close()
    time.sleep(20000/1000000.0) #20ms
    prevState = actState

def wheels_thread2():
  prevState = GPIO.input(WHEELS_PIN2)
  counter = 0
  while True:
    actState = GPIO.input(WHEELS_PIN2)
    #if actState:
    #  print "1"
    #else:
    #  print "0"
    if ((not prevState) and (actState)):
      time.sleep(3000/1000000.0) #3ms
      actState = GPIO.input(WHEELS_PIN2)
      if (actState):
        counter += 1
        print "[2]Liczba zboczy: " + str(counter)
        fsock = open(WHEELS_FILE2, 'a')
        fsock.write(str(counter) + "\n")
        fsock.close()
    time.sleep(20000/1000000.0) #20ms
    prevState = actState

def sigterm_handler(_signo, _stack_frame):
  print "test"
  server_sock.close()
  print "all done"

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

print "START"
signal(SIGTERM, sigterm_handler)
start_new_thread(gps_thread, ())
start_new_thread(wheels_thread, (WHEELS_PIN1, WHEELS_FILE1, ))
start_new_thread(wheels_thread, (WHEELS_PIN2, WHEELS_FILE2, ))
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

