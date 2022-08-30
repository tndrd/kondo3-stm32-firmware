from kondo_controller import Rcb4BaseLib
import time
from queue import Queue
from math import sin
import RPi.GPIO as GPIO
from threading import Thread

openmv = False


class STM32Package():
    def __init__(self): pass
    
    class PERIPHERY_ADDR:
        KONDO = 0x03

    class CONTROL_BYTES:
        SOM = bytearray(b'\xF0\xAA')

    def pack(periph_addr, txBuf, txLen, rxLen):
        package = bytearray(STM32Package.CONTROL_BYTES.SOM)
        package.append(txLen)
        package.append(rxLen)
        package.append(periph_addr)
        package.extend(bytearray(txBuf))
        return package

class Quaternion:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.w = None
        self.time = None
        
    def read(self, byte):
        x = ((byte[0])<<8) | ((byte[1])<<0) 
        y = ((byte[2])<<8) | ((byte[3])<<0)
        z = ((byte[4])<<8) | ((byte[5])<<0)
        w = ((byte[6])<<8) | ((byte[7])<<0)
        time = (byte[8]<<56) | (byte[9]<< 48) | (byte[10]<< 40) | (byte[11]<< 32) | (byte[12]<< 24) | (byte[13]<< 16) | (byte[14]<< 8) | (byte[15])
        #python думает что компоненты x, y, z, w являются беззнаковыми переменными
        #поэтому переводим эти переменные в signed int тип данных
        if(x & 0x8000):
            x = -0x10000 + x
        if(y & 0x8000):
            y = -0x10000 + y
        if(z & 0x8000):
            z = -0x10000 + z
        if(w & 0x8000):
            w = -0x10000 + w
        
        self.x = float(x) / 16384
        self.y = float(y) / 16384
        self.z = float(z) / 16384
        self.w = float(w) / 16384
        self.time = time

    def dump(self):
        print('x = ', self.x)
        print('y = ', self.y)
        print('z = ', self.z)
        print('w = ', self.w)
        print('time in nanoseconds= ',(self.time))
        print('time in seconds=     ',(self.time / 1000000000))


class Kondo3Rcb4BaseLib(Rcb4BaseLib):
    def __init__(self):
        super().__init__()
        self._init_STM32()
        self._rx_income = Queue(maxsize=500)
        self._tx_queue  = Queue(maxsize=500)
        self._rx_active = 1
        self._tx_active = 1
        
        self._rx_free = 1
        self.robot_state = self.RobotState()
        
        self._read_thread = Thread(target=self._read_loop)
        self._send_thread = Thread(target=self._send_loop)
        
        self._read_thread.start()
        self._send_thread.start()
        
    class RobotState:
        def __init__(self):
            self.IMU_3    = None
            self.IMU_4    = None
            self.IMU_HEAD = Quaternion()
        
        def dump(self):
            print(f"\nState: {self.IMU_3} {self.IMU_4}")
            self.IMU_HEAD.dump()
            print("")
            
    def synchronize(self, txBuf, rxLen, writeOnly=False):
        sendbuf = [256]
        if self.isSynchronize == False:
            sendbuf.clear()
            if (len(txBuf) == 0 or rxLen <= 3):
                rxbuf = []
                return rxbuf

            for i in range(len(txBuf)):
                if txBuf[i] < 0 or 255 < txBuf[i]:
                    rxbuf = []
                    return rxbuf
                sendbuf.append(txBuf[i])

            self.isSynchronize = True
            
            if writeOnly:
                self._tx_queue.put(self.tx_pack(sendbuf, rxLen))
                self._rx_income.put(rxLen)
                self.isSynchronize = False
                return []
                        
            while not (self._rx_income.empty() and self._rx_free): pass
            self._tx_active = False
            
            self.com.write(self.tx_pack(sendbuf, rxLen))
            #print(f"Sent      {', '.join(hex(b) for b in bytearray(sendbuf))}")
            rxbuf = self.com.read(rxLen)    
            #print(f"Recived   {', '.join(hex(b) for b in bytearray(rxbuf))}")
            
            self._rx_income.put(0)
            
            self._tx_active = True
            
            if rxbuf is not None and len(rxbuf) != 0:
                if self.checkCheckSum(rxbuf) == False:
                    rxbuf = []
            else:
                rxbuf = []
            self.isSynchronize = False
            return rxbuf
        else:
            rxbuf = []
            return rxbuf

    def tx_pack(self, txBuf, rxLen):
        kondo_addr = STM32Package.PERIPHERY_ADDR.KONDO
        return STM32Package.pack(kondo_addr, txBuf, len(txBuf), rxLen)
        
    def _reader(self):
        if self._rx_active and not self._rx_income.empty():
            self._rx_free = 0
            rxLen = self._rx_income.get()
            
            rxbuf = self.com.read(rxLen + 5 + 5 + 16)
            
            self.robot_state.IMU_3 = rxbuf[rxLen:rxLen+5]
            self.robot_state.IMU_4 = rxbuf[rxLen+5:rxLen+10]
            serialized_quaternion  = rxbuf[rxLen+10:]
            self.robot_state.IMU_HEAD.read(serialized_quaternion)
            
            self._rx_free = 1
    
    def _sender(self):
        if self._tx_active and not self._tx_queue.empty():
            sendbuf = self._tx_queue.get()
            self.com.write(sendbuf)
            #print(f"Sent      {', '.join(hex(b) for b in bytearray(sendbuf))}")

    def _send_loop(self):
        while True: self._sender()

    def _read_loop(self):
        while True: self._reader()

    def setServoPosAsync (self,servoDatas,frame):
        rxLen, txbuf = self.runConstFrameServoCmd(servoDatas,frame)
        return self.synchronize(txbuf, rxLen, writeOnly=True)
    
    def _unreset_STM32(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        STM32_RST_PIN = 6
        GPIO.setup(STM32_RST_PIN,GPIO.OUT) # пин RST для STM32
        GPIO.output(STM32_RST_PIN, 0)    # устанавливаем пин RST для STM32 в лог 0

    def _reset_STM32(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        STM32_RST_PIN = 6
        GPIO.setup(STM32_RST_PIN,GPIO.OUT) # пин RST для STM32
        GPIO.output(STM32_RST_PIN, 1)    # устанавливаем пин RST для STM32 в лог 0

    def _init_STM32(self, time_gap = 0.5):
        print("Init STM32...", end=" ")
        self._reset_STM32()
        time.sleep(time_gap)
        self._unreset_STM32()
        time.sleep(time_gap)
        print("Done\n")
    
if __name__ == "__main__":
    
    kondo = Kondo3Rcb4BaseLib()
    kondo.open('/dev/ttyAMA2', 1250000, 10)
    
    sd = kondo.ServoData()
    sd.id   = 9
    sd.sio  = 1 
    sd.data = 7500
    
    sd2 = kondo.ServoData()
    sd2.id   = 8
    sd2.sio  = 1
    sd2.data = 7500
    
    for i in range(20):
        print(f"Iteration {i+1}")
        x=0
        while x<500:
            sd.data = 7500 + int(500 * sin(0.05*x))
            kondo.setServoPosAsync([sd], 5)
            x+=1
    
        print("")
        kondo.robot_state.dump()
        print(kondo.getSinglePos(9, 1))
        
        x=0
        while x<500:
            sd.data = 7500 + int(250 * sin(0.05*x))
            kondo.setServoPosAsync([sd], 5)
            x+=1
        
        print("")
        kondo.robot_state.dump()
        print(kondo.getSinglePos(9, 1))
        
        x=0
        while x<500:
            sd.data = 7500 + int(1000 * sin(0.05*x))
            kondo.setServoPosAsync([sd], 1)
            x+=1
        
        print("")
        kondo.robot_state.dump()
        print(kondo.getSinglePos(9, 1))
        
        time.sleep(6)
        
    time.sleep(5)
    print(kondo.getSinglePos(9, 1))
    print(kondo.getSinglePos(9, 1))
    print(kondo.getSinglePos(9, 1))