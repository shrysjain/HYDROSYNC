"""
Water Level Event Handler
June 2024
Shreyas Jain, Arjun Kode, Bera Gumruk
"""

# Install dependencies
from serial import Serial # type: ignore
from time import sleep
from requests import post
import subprocess

# Open serial monitor
ser = Serial("/dev/cu.usbmodem11301", 115200)
sleep(2)

# Notification helpers
def low_water():
  post("https://ntfy.sh/hydrosync",
    data="Looks like you are running low on water! Refill your water bottle soon to stay hydrated",
    headers={
        "Title": "Low water detected",
        "Priority": "urgent",
        "Tags": "warning"
    })

# Reminders
subprocess.Popen(["python", "reminderHandler.py"])

# Events
while True:
  sleepTime = 1
  value = int(ser.readline().decode())
  
  print(value)
  
  if value == 0:
    low_water()
    sleepTime = 60
  
  sleep(sleepTime)
