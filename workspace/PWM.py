import RPi.GPIO as GPIO
from time import sleep

PWM_PIN_M1 = 7 #7 #11
DIR_PIN1_M1 = 19 # 10
DIR_PIN2_M1 = 21 # 9



#PWM_PIN_M2 = 
DIR_PIN1_M2 = 13 
DIR_PIN2_M2 = 15

#start_new_thread(changePWM, ())
#GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)

#GPIO.cleanup()

GPIO.setup(PWM_PIN_M1, GPIO.OUT)
GPIO.setup(DIR_PIN1_M1, GPIO.OUT)
GPIO.setup(DIR_PIN2_M1, GPIO.OUT)


#GPIO.setup(PWM_PIN_M2, GPIO.OUT)
GPIO.setup(DIR_PIN1_M2, GPIO.OUT)
GPIO.setup(DIR_PIN2_M2, GPIO.OUT)

#while 1:
GPIO.output(DIR_PIN1_M1, True)#  GPIO.output(DIR_PIN2_M1, False)
GPIO.output(DIR_PIN2_M1, False)


GPIO.output(DIR_PIN1_M2, True)#  GPIO.output(DIR_PIN2_M1, False)
GPIO.output(DIR_PIN2_M2, True)
sleep(1)
#GPIO.output(PWM_PIN_M1, True)
  #GPIO.output(PWM_PIN_M1, False)
  #GPIO.output(DIR_PIN2_M1, True)
  #sleep(1)
#GPIO.output(10, True)
#GPIO.output(9, False)


p = GPIO.PWM(PWM_PIN_M1,500)


p.start(100)
pause_time = 0.04

#GPIO.cleanup()
pwm = 75
while True:  
  data = raw_input()
  if data == "end":
    print "END"
    break
  elif data == "w":
    #global pwm
    pwm+=1
    if pwm > 100:
      pwm = 100
    print pwm
    p.ChangeDutyCycle(pwm)  
  elif data == "s":
    #global pwm
    pwm-=1
    if pwm < 0:
      pwm = 0
    print pwm
    p.ChangeDutyCycle(pwm)  
  elif data == "e":
    #global pwm
    pwm+=5
    if pwm > 100:
      pwm = 100
    print pwm
    p.ChangeDutyCycle(pwm)  
  elif data == "d":
    #global pwm
    pwm-=5
    if pwm < 0:
      pwm = 0
    print pwm
    p.ChangeDutyCycle(pwm)
  elif data == "j":
    GPIO.output(DIR_PIN1_M1, True)#  GPIO.output(DIR_PIN2_M1, False)
    GPIO.output(DIR_PIN2_M1, False)
  elif data == "k":
    GPIO.output(DIR_PIN1_M1, True)#  GPIO.output(DIR_PIN2_M1, False)
    GPIO.output(DIR_PIN2_M1, True)
  elif data == "l":
    GPIO.output(DIR_PIN1_M1, False)#  GPIO.output(DIR_PIN2_M1, False)
    GPIO.output(DIR_PIN2_M1, True)
#p.ChangeFrequency(100)

#raw_input()
#p.stop()
#serv1.stop()
#serv2.stop()

GPIO.cleanup()
