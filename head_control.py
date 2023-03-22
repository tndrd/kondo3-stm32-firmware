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

from queue import Queue


DEFAULT_PORT = '/dev/ttyAMA2'
DEFAULT_BAUD = 115200
DEFAULT_TOUT = 5

PAN_ID = 10
TLT_ID = 31

TIMESTEP = 0.02 # 20 ms

MIN_SPEED = 50
MAX_SPEED = 100
SPEED = 300

P_LIMITS = (5500, 9500)
T_LIMITS = (6000, 7900)

STEP_TOLERANCE = 500

def checkFirmware():
    ENVV = "FIRMWARE_STARTED"
    if ENVV not in os.environ:
        print("Error: Firmware not started")
        return False
    if os.environ[ENVV] != "1":
        print("Error: Firmware not started")
        return False
    
    return True
    
    deinitSTM()
    time.sleep(0.2)
    initSTM()
    time.sleep(1)
    print("Starting Firmware...")
    ser = serial.Serial(BOOT_PORT, BOOT_BAUD)
    ser.write("CMD:START_FW\r\n".encode('ascii'))
    #print(ser.readline())
    print("Done")      
    

class HeadController:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=DEFAULT_TOUT):
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._com = serial.Serial(port, baud, timeout=timeout)
        self._active = False

        self._start_pan = 7500
        self._start_tilt = 7500

        self._goal_pan  = 7500
        self._goal_tilt = 7500

        self._pan = 7500
        self._tilt = 7500

        self._thread = Thread(target=self._loop)
        
        self._min_speed = MIN_SPEED
        self._max_speed = MAX_SPEED
        #self.speedConst = SPEED
        self._pan_ready  = True
        self._tlt_ready = True 

    def start(self):
        if not self._active:
            self._active = True
            print("Starting head_controller", end="")
            #p, t = self._send_p_t_sync(7500, 7500)          
            #print(int(p), int(t))

            self._pan_ready = True
            self._tlt_ready = True

            self.set_pan(7500, True)
            self.set_tilt(7500, True)

            self._thread.start()

            while not self._pan_ready or not self._tlt_ready:
              print(".", end="")
              time.sleep(1)
            print(" Done")


    def stop(self):
        if self._active:
            print("Stopping HeadController...")
            self._active = False

            self._pan_ready = False
            self._tlt_ready = False

            self._thread.join()
            self._send_p_t(0,0)
            time.sleep(1)
            self._com.flushInput()
            self._com.close()
            print("HeadController stopped")

    def __del__(self):
        print("Destructing BaseHeadController")
        self.stop()

    def _stm_cmd(self, txBuf, txLen, rxLen):
        buff = bytearray(b'\xF0\xAA')
        buff.append(txLen)
        buff.append(rxLen)
        buff.append(0x03)
        buff.extend(bytearray(txBuf))
        return buff

    def _pos_cmd(self, id, pos):
        txbuf = []
    
        cmd = 0x80 | (id & 0x1F)# id
        txbuf.append(cmd)
    
        txbuf.append((pos >> 7) & 0x7F)
        txbuf.append(pos & 0xff)
        return txbuf

    def _send_pos(self, id, pos):
        pos_cmd = self._pos_cmd(id, pos)
        stm_cmd = self._stm_cmd(pos_cmd, len(pos_cmd), 6)
        self._com.write(stm_cmd) 

    def _send_p_t(self, pan, tilt):
        self._send_pos(TLT_ID, tilt)
        time.sleep(TIMESTEP)
        self._send_pos(PAN_ID, pan)
        time.sleep(TIMESTEP)

    def _send_p_t_sync(self, pan, tilt):
        self._send_pos(TLT_ID, tilt)
        t = self._com.read(6)
        time.sleep(TIMESTEP)
        self._send_pos(PAN_ID, pan)
        p = self._com.read(6)
        time.sleep(TIMESTEP)
        return p, t

    def _step(self, start, val, goal):
        if abs(val - goal) < self._min_speed:
            return goal

        pos = float(val - start) / float(goal - start)
        
        sgn = (goal - start) / abs(goal - start)
        
        dv = self._min_speed + (self._max_speed * (1 - abs(2 * (0.5 - pos))))

        return int(val + dv * sgn)
    def _step_new(self, start, val, goal):
        if abs(goal - start) == 0: return int(val)
        if abs(goal - val) < 100: return int(goal)

        sgn = int((goal - start) / abs(goal - start))
        center = (goal+start)/2
        dv = abs((val-start)*(val-goal)/((center-goal)*(center-start)))
        print(dv)
        
        #time.sleep(1)

        print(val,goal,start)
        print()


        #print(dv)

        #print(int(val+dv*sgn*self.speedConst))

        return int(goal)

    def _loop(self):
        while(self._active):
            self._tilt = self._step(self._start_tilt, self._tilt, self._goal_tilt)
            self._pan  = self._step(self._start_pan, self._pan, self._goal_pan)
            
            self._send_p_t(self._pan, self._tilt)
            self._com.flushInput()
            
            self._update_state()
            #print(f"T: {self._tilt} | P: {self._pan}")
    
    def _update_state(self):
        if self._tilt == self._goal_tilt:
                self._tlt_ready = True
        if self._pan == self._goal_pan:
                self._pan_ready = True
    
    def _limit(self, pos, limit):
        if pos > limit[1]: return limit[1]
        if pos < limit[0]: return limit[0]
        return pos

    def set_pan(self, pos, force=False):
        if not force and abs(self._pan - pos) < STEP_TOLERANCE:
            return
        
        if not self._pan_ready:
            print("Error: attempt to set pan while servo is busy")
            return
        
        self._pan_ready = False
        
        self._start_pan = self._pan
        self._goal_pan = self._limit(pos, P_LIMITS)

    def set_tilt(self, pos, force=False):
        if not force and abs(self._tilt - pos) < STEP_TOLERANCE:
            return
        
        if not self._tlt_ready:
            print("Error: attempt to set tilt while servo is busy")
            return
        
        self._tlt_ready = False
        
        self._start_tilt = self._tilt
        self._goal_tilt = self._limit(pos, T_LIMITS)

    def set_speed(self, max_speed, min_speed = MIN_SPEED):
        self._min_speed = min_speed
        self._max_speed = max_speed
    def set_speed_const(self, speedConst):
        self.speedConst = speedConst

    def tilt_ready(self):
        return self._tlt_ready
    
    def pan_ready(self):
        return self._pan_ready


WAIT_STEP = 0.01

class AsyncHeadController(HeadController):
    def __init__(self):
        super().__init__()

        self._pan_queue = Queue()
        self._tlt_queue = Queue()
        
        self._qthread = Thread(target=self._qloop)
        
        self._qactive = False
        
    def start(self):
        if not self._qactive:
            super().start()
            self._qactive = True
            self._qthread.start()
    
    def stop(self):
        if self._qactive:
            print("Stopping AsyncHeadController...")
            self._qactive = False
            self._qthread.join()
            super().stop()
            print("AsyncHeadController stopped")
            
    
    def __del__(self):
        print("Destructing AsycHeadController")
        self.stop()
        
    
    def set_pan(self, pos, force=False):
        while not self._pan_queue.empty():
            time.sleep(WAIT_STEP)
        self.add_pan(pos)
    
    def set_tilt(self, pos, force=False):
        while not self._tlt_queue.empty():
            time.sleep(WAIT_STEP)
        self.add_tilt(pos)
    
    def set(self, pan, tilt):
        while not self._pan_queue.empty() or not self._tlt_queue.empty():
            time.sleep(WAIT_STEP)
        self.add_pan(pan)
        self.add_tilt(pan)
    
    def add_pan(self, pos):
        self._pan_queue.put(pos)

    def add_tilt(self, pos):
        self._tlt_queue.put(pos)
        
    def _qloop(self):
        while(self._qactive):
            if super().pan_ready() and not self._pan_queue.empty():
                pan = self._pan_queue.get()
                super().set_pan(pan)
            if super().tilt_ready() and not self._tlt_queue.empty():
                tilt = self._tlt_queue.get()
                super().set_tilt(tilt)
            time.sleep(WAIT_STEP)

def get_controller():
    if not checkFirmware():
        raise RuntimeError("Firmware not Started. Run start_fw.sh to turn firmware on")
    
    control = AsyncHeadController()
    control.start()
    return control