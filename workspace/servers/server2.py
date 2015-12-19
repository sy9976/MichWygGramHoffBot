#!/usr/bin/python
from thread import *
from bluetooth import *
from signal import *
import gps

server_sock = None
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

def sigterm_handler(_signo, _stack_frame):
  print "test"
  server_sock.close()
  print "all done"

def main_thread(client_sock):

  try:
    while True:
      data = client_sock.recv(1024)
      if data == "gps":
        while True:
          report = session.next()
          if report['class'] == 'TPV':
            if hasattr(report, 'time'):
              print "czas:",report.time
            if hasattr(report, 'lat'):
              print "\tszerokosc:\t",report.lat
              gps_data = "szerokosc " + str(report.lat)
              #client_sock.send(gps_data)
            if hasattr(report, 'lon'):
              print "\tdlugosc:\t",report.lon
              gps_data = "dlugosc " + str(report.lon)
              #client_sock.send(gps_data)
              #break
            if hasattr(report, 'speed'):
              print "\tszybkosc:\t",report.speed
          if (hasattr(report, 'lat') and hasattr(report, 'lot')):
            gps_data = "" + report.lat + " " + report.lot
            client_sock.send(gps_data)
            break
          #else:
            #client_sock.send("GPS NIE DZIALA, ALE LEGIA MISTRZEM")
      if len(data) == 0: break
      print "received [%s]" % data
  except IOError:
    pass

  print "disconnected"

  client_sock.close()
  #server_sock.close()

print "START"
signal(SIGTERM, sigterm_handler)
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

  '''try:
  while True:
  data = client_sock.recv(1024)
  if len(data) == 0: break
  print "received [%s]" % data
  except IOError:
  pass

  print "disconnected"

  client_sock.close()
  server_sock.close()
  print "all done"'''
