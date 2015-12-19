import RPi.GPIO as GPIO
from time import sleep
from thread import *
WHEELS_PIN1 = 16
WHEELS_PIN2 = 18

GPIO.setmode(GPIO.BOARD)
GPIO.setup(WHEELS_PIN1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(WHEELS_PIN2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

'''def wheels_thread1():
  prevState = GPIO.input(WHEELS_PIN1)
  counter = 0
  while True:
    actState = GPIO.input(WHEELS_PIN1)
    #if actState:
    #  print "1"
    #else:
    #  print "0"
    if ((not prevState) and (actState)):
      sleep(3000/1000000.0) #5ms
      actState = GPIO.input(WHEELS_PIN1)
      if (actState):
        counter += 1
        print "[1]Liczba zboczy: " + str(counter)
        fsock = open('/home/pi/Desktop/MichWygGramHoffBot/log/wheels1.txt', 'a')
        fsock.write(str(counter) + "\n")
        fsock.close()
    sleep(200000/1000000.0) #50ms
    prevState = actState

start_new_thread(wheels_thread1, ())'''
prevState = GPIO.input(WHEELS_PIN2)
counter = 0
while True:
  actState = GPIO.input(WHEELS_PIN2)
  #if actState:
  #  print "1"
  #else:
  #  print "0"
  if ((not prevState) and (actState)):
    sleep(3000/1000000.0) #5ms
    actState = GPIO.input(WHEELS_PIN2)
    if (actState):
      counter += 1
      print "[2]Liczba zboczy: " + str(counter)
      fsock = open('/home/pi/Desktop/MichWygGramHoffBot/log/wheels2.txt', 'a')
      fsock.write(str(counter) + "\n")
      fsock.close()
  sleep(20000/1000000.0) #50ms
  prevState = actState




