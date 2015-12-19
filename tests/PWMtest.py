import RPi.GPIO as GPIO
from time import sleep

PWM_PIN = 7 #11
DIR_PIN1 = 19 # 10
DIR_PIN2 = 21 # 9

#GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)

#GPIO.cleanup()

GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN1, GPIO.OUT)
GPIO.setup(DIR_PIN2, GPIO.OUT)

#GPIO.setup(23, GPIO.OUT)#bcm 11
#GPIO.setup(19, GPIO.OUT)#bcm 10
#GPIO.setup(21, GPIO.OUT)#bcm 9
#while 1:
GPIO.output(DIR_PIN1, True)#  GPIO.output(DIR_PIN2, False)
GPIO.output(DIR_PIN2, False)
sleep(1)
#GPIO.output(PWM_PIN, True)
  #GPIO.output(PWM_PIN, False)
  #GPIO.output(DIR_PIN2, True)
  #sleep(1)
#GPIO.output(10, True)
#GPIO.output(9, False)


#serv1 = GPIO.PWM(9, 500)
#serv1.start(100)

#serv2 = GPIO.PWM(10, 500)
#serv2.start(0)

p = GPIO.PWM(PWM_PIN,500)


p.start(10)
pause_time = 0.04

#GPIO.cleanup()
while True:  
  for i in range(0,101):      # 101 because it stops when it finishes 100  
    p.ChangeDutyCycle(i)  
    sleep(pause_time)#p.ChangeDutyCycle(90)
#p.ChangeFrequency(100)

raw_input()
#p.stop()
#serv1.stop()
#serv2.stop()

GPIO.cleanup()
