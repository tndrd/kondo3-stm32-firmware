openmv = False
import time
import struct
import math as m

from enum import Enum
import serial
from struct import *
import RPi.GPIO as GPIO
from math import sin
from threading import Thread
import os

BOOT_PORT = '/dev/ttyAMA2'
BOOT_BAUD = 921600

def initSTM():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    STM32_RST_PIN = 6
    GPIO.setup(STM32_RST_PIN,GPIO.OUT) # пин RST для STM32
    GPIO.output(STM32_RST_PIN, 0)    # устанавливаем пин RST для STM32 в лог 0

def deinitSTM():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    STM32_RST_PIN = 6
    GPIO.setup(STM32_RST_PIN,GPIO.OUT) # пин RST для STM32
    GPIO.output(STM32_RST_PIN, 1)    # устанавливаем пин RST для STM32 в лог 0

def startFirmware():
    ENVV = "FIRMWARE_STARTED"
    print("Starting Firmware... ", end="")
    
    if ENVV in os.environ:
        if os.environ[ENVV] == "1":
            print("Already Started")
            return
    
    deinitSTM()
    time.sleep(0.2)
    initSTM()
    time.sleep(1)
    
    ser = serial.Serial(BOOT_PORT, BOOT_BAUD)
    ser.write("CMD:START_FW\r\n".encode('ascii'))
    
    print("Done")      

if __name__ == "__main__":
    startFirmware()