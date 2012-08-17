#!/usr/bin/python
import os,time,subprocess

HYDRA_INDEX = -1

cmd = "ls /dev/hidraw*"
p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout #cmd is your script
devices = p.readlines()
# What if there are no hidraw devices?

for i in range(len(devices)):
  cmd = "cat /sys/class/hidraw/hidraw"+str(i)+"/device/uevent"
  p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout #cmd is your script
  lines= p.readlines()
  #print lines
  if 'HID_NAME=Razer Razer Hydra\n' in lines:
    HYDRA_INDEX = i
    break


print "device %d is the Razer Hydra"%(HYDRA_INDEX)
