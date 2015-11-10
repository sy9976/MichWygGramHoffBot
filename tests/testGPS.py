#-*- coding: utf-8 -*-
import gps
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE )
#print "TEST"
while True:
  #print "TEST1"
  report = session.next()
  #klasa TPV zawiera kilka ciekawych informacji
  if report['class'] == 'TPV':
    if hasattr(report, 'time'):
      print "czas:",report.time
    if hasattr(report, 'lat'):
      print "\tszerokosc:\t",report.lat
    if hasattr(report, 'lon'):
      print "\tdlugosc:\t",report.lon
    if hasattr(report, 'speed'):
      print "\tszybkosc:\t",report.speed
  #tu znajdują się satelity i parametry transmisji
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
