#!/bin/bash
DATE=$(date -d "today" +"%Y-%m-%d %H:%M")
LOG_STARTUP_PATH=/home/pi/Desktop/MichWygGramHoffBot/log/startup_"$USER"_"$DATE".log
LOG_SERVER_PATH=/home/pi/Desktop/MichWygGramHoffBot/log/server_"$USER"_"$DATE".log
#LOG_WHEELS_PATH=/home/pi/Desktop/MichWygGramHoffBot/log/wheelsRotation_"$USER"_"$DATE".log
touch "$LOG_STARTUP_PATH"
touch "$LOG_SERVER_PATH"
echo "-----------START------------" >> "$LOG_STARTUP_PATH"
date +"%T %d-%m-%Y" >> "$LOG_STARTUP_PATH"
echo "LOG_STARTUP_PATH:" $LOG_STARTUP_PATH >> "$LOG_STARTUP_PATH"
echo "Startup script [tutaj data]" >> "$LOG_STARTUP_PATH"
echo "[STATUS] before enabling" >> "$LOG_STARTUP_PATH"
hciconfig 1>>"$LOG_STARTUP_PATH" 2>>"$LOG_STARTUP_PATH"
echo "hci enabling" >> "$LOG_STARTUP_PATH"
rfkill unblock bluetooth 1>>"$LOG_STARTUP_PATH" 2>> "$LOG_STARTUP_PATH"
sleep 20 
sudo hciconfig hci0 up 2>> "$LOG_STARTUP_PATH"
sudo hciconfig hci0 piscan 2>> "$LOG_STARTUP_PATH"
echo "hci enabled" >> "$LOG_STARTUP_PATH"
echo "[STATUS] after enable" >> "$LOG_STARTUP_PATH"
hciconfig 1>>"$LOG_STARTUP_PATH" 2>>"$LOG_STARTUP_PATH"

echo "server startup time" >> "$LOG_SERVER_PATH"
date +"%T %d-%m-%Y" >> "$LOG_SERVER_PATH"
sudo python /home/pi/Desktop/MichWygGramHoffBot/src/server.py 1>>"$LOG_SERVER_PATH"
#sudo python /home/pi/Desktop/MichWygGramHoffBot/src/ReadINPin.py 1>>"$LOG_WHEELS_PATH"

echo "----------END-----------" >> "$LOG_STARTUP_PATH"
echo " " >> "$LOG_STARTUP_PATH"

#echo "KONIEC KURWA ZABAWY"
