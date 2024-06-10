"""
Reminder Event Handler
June 2024
Shreyas Jain, Arjun Kode, Bera Gumruk
"""

# Install dependencies
from time import sleep
from requests import post

# Notification helpers
def reminder():
  post ("https://ntfy.sh/hydrosync",
      data="It's been a while since you have last had water. Consider taking a sip out of your water bottle :)",
      headers={
          "Title": "Reminder",
          "Priority": "default",
          "Tags": "droplet"
      })

# Events
while True:
  reminder()
  sleep(1800)
