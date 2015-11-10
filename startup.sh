#!/bin/bash
LOG_PATH=/home/pi/Desktop/robot/startup_"$USER".log
echo "-----------START------------" >> "$LOG_PATH"
echo "LOG_PATH:" $LOG_PATH >> "$LOG_PATH"
echo "Startup script [tutaj data]" >> "$LOG_PATH"
echo "[STATUS]" >> "$LOG_PATH"
hciconfig 1>>"$LOG_PATH" 2>>"$LOG_PATH"
echo "hci enabling" >> "$LOG_PATH"
rfkill unblock bluetooth 1>>"$LOG_PATH" 2>> "$LOG_PATH"
sleep 20 
sudo hciconfig hci0 up 2>> "$LOG_PATH"
sudo hciconfig hci0 piscan 2>> "$LOG_PATH"
echo "hci enabled" >> "$LOG_PATH"
echo "[STATUS]" >> "$LOG_PATH"
hciconfig 1>>"$LOG_PATH" 2>>"$LOG_PATH"
#echo "END2"
#python script.py  #tutaj Przemek podpinasz skrypcik


echo "----------END-----------" >> "$LOG_PATH"
echo " " >> "$LOG_PATH"


#echo "KONIEC KURWA ZABAWY"
