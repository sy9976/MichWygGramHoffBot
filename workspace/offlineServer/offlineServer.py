#!/usr/bin/python
from thread import *
from bluetooth import *
from signal import *
import gps
import time
import RPi.GPIO as GPIO

server_sock = None
lat = 0
lon = 0
cTime = None

session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
GPIO.setmode(GPIO.BOARD)

def sigterm_handler(_signo, _stack_frame):
  print "test"
  server_sock.close()
  print "all done"

'''def gps_thread():
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
'''
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

f = open('points.txt', 'r')
print f.readline()
print f.readline()
'''while True:

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
'''
