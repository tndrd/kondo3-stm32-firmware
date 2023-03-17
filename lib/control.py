openmv = False
import time
import struct

from enum import Enum
import serial
from struct import *
import RPi.GPIO as GPIO
from math import sin

DEFAULT_PORT = '/dev/ttyAMA2'
DEFAULT_BAUD = 1250000
DEFAULT_TOUT = 1.3

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

class HeadController:
  def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=DEFAULT_TOUT):
    initSTM()
    self._port = port
    self._baud = baud
    self._timeout = timeout
    self._com = serial.Serial(port, baud, timeout)

  def close(self):
    deinitSTM()
    try:
      self._com.close()
      self._com = 0
      return 0
    except:
      return -1

  def STMKondoWrite(self, txBuf, txLen, rxLen):
    buff = bytearray(b'\xF0\xAA')
    buff.append(txLen)
    buff.append(rxLen)
    buff.append(0x03)
    buff.extend(bytearray(txBuf))
    self._com.write(buff)

  def _write(self, id, pos):
    txbuf = []
    
    cmd = 0x80 | (id & 0x1F)# id
    txbuf.append(cmd)
    
    txbuf.append((pos >> 7) & 0x7F)
    txbuf.append(pos & 0xff)

    self.com.flushInput()
    self.STMKondoWrite(txbuf, len(txBuf), 6)

  def set_pan(self, pos):
    self._write(0, pos)

  def set_tilt(self, pos):
    self._write(1, pos)

  
