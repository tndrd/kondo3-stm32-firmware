openmv = False
import time
import struct

from enum import Enum
import serial
from struct import *
import RPi.GPIO as GPIO
from math import sin
# try:
#     from pyb import UART
# except ImportError:
#     raise Exception("Trying to import MicroPython library using Python3")


# from micropython import const

# class Serial:
#     def __init__(self, uart):
#         self.uart = uart

#     def flushInput(self):
#         if openmv:
#             buf = self.uart.any()
#             self.uart.read(buf)
#         else:
#             self.uart.flushInput()

#     def read(self, rxLen):
#         if openmv:
#             msg = []
#             for _ in range(rxLen):
#                 msg.append(self.uart.readchar())
#             #print("read ",msg)
#             return bytes(bytearray(msg))
#         else:
#             #print("read ",msg)
#             return self.uart.read(rxLen)

#     def write(self, buf):
#         if openmv:
#             tmp = bytes(bytearray(buf))
#             #print("written ",buf)
#             for b in tmp:
#                 self.uart.writechar(b)
#                 # time.sleep(1)
#         else:
#             #print("written ",buf)
#             self.uart.write(buf)

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


class Rcb4BaseLib:
    def __init__(self):
        self.maxMotionCount = (120)
        self.MotionSingleDataCount = 2048
        self.UserParameterSingleDataCount = 2
        self.UserParameterCount = 20
        self.IcsDeviceSize = 35
        self.IcsDeviceDataSize = 20
        self.CounterSingleDataCount = 1
        self.CounterCount = 10
        self.AdcCount = 11
        self.AdcDataCount = 22
        self.__isSynchronize = False
        self.__configData = 0
        self.comOpen = False
        self.com = None



    def open(self,comName,bundrate,timOut):
        if self.com == None:
            try:
                self.com = serial.Serial(comName,bundrate,timeout=timOut)
                self.com.flushInput()#
                
                if self.checkAcknowledge() == True:
                    #ACKが成功した場合Configデータを取得
                    confData = self.getConfig()
                    #confDataは0xFFFFの場合はエラー
                    if(confData == 0xFFFF):
                        return False
                    else:
                        self.__configData = confData
                        return True
                else:
                    return False
                                
            except:        
                return False
        else:
            return False

    
    def close(self):
        try:
            self.com.close()
            self.com = 0
            return 0
        except:
            return -1


    @staticmethod
    def CheckSum(dataBytes):
        sum = 0
        for i in range(dataBytes[0] - 1):
            sum = sum + dataBytes[i]
        return sum & 0xff

    def __checkCheckSum(self, dataBytes):
        if dataBytes[0] == 0:
            return False
        else:
            if dataBytes[dataBytes[0] - 1] == self.CheckSum(dataBytes):
                return True
            else:
                return False

    class CmdOkType:
        Ok = 0
        Error = -1

    class AckType:
        Ack = 0x06
        Nack = 0x15

    class SubMoveCmd:
        RamToCom = 0x20
        ComToRam = 0x02
        DeviceToCom = 0x21
        ComToDevice = 0x12

    class CommandTypes:
        Move = 0x00
        Call = 0x0C
        AckCheck = 0xFE
        SingleServo = 0x0F
        ConstFrameServo = 0x10
        ServoParam = 0x12

    class RamAddr:
        ConfigRamAddress = 0x0000
        ProgramCounterRamAddress = 0x0002
        UserParameterRamAddress  = 0x0462
        CounterRamAddress        = 0x0457
        AdcRamAddress            = 0x0022
        PioModeAddres	         = 0x0038 
        PioAddress               = 0x003A

    class RomAddr:
        StartupCmdRomAddress = 0x0444
        MainLoopCmd          = 0x044B
        MotionRomAddress     = 0x0b80

    class ConfigData:
        EnableSio = (0x0001)
        EnableRunEeprom = (0x0002)
        EnableServoResponse = (0x0004)
        EnableReferenceTable = (0x0008)

    class ServoData:
        def __init__(self, id=0, sio=0, data=0):
            self.id = id
            self.sio = sio
            self.data = data

        def icsNum2id(self):
            return self.id * 2 + (self.sio - 1)

        def itemAdd(self, x, y, z):
            self.id = x
            self.sio = y
            self.data = z

        def __lt__(self, other):
            return (self.id * 2 + (self.sio - 1)) < (other.id * 2 + (other.sio - 1))
        
        def __repr__(self):
            return "({0}, {1}, {2})".format(self.id, self.sio, self.data)

    class DeviceAddrOffset:
        CategoryAddressOffset      = 0x00
        IDAddressOffset            = 0x01
        TrimAddressOffset          = 0x02
        MotorPositionAddressOffset = 0x04
        PositionAddressOffset      = 0x06
        FrameAddressOffset         = 0x08
        Mixing1AddressOffset       = 0x0E
        Mixing1RatioAddressOffset  = 0x10
        Mixing2AddressOffset       = 0x11
        Mixing2RatioAddressOffset  = 0x13


    def STMKondoWrite(self, txBuf, txLen, rxLen):
        buff = bytearray(b'\xF0\xAA')
        buff.append(txLen)
        buff.append(rxLen)
        buff.append(0x03)
        buff.extend(bytearray(txBuf))
        self.com.write(buff)
#         self.com.write(b'\xF0\xAA\x04\x04\x03\x04\xfe\x06\x08')

    def synchronize(self, txBuf, rxLen, writeOnly=False):
        sendbuf = [256]
        if self.__isSynchronize == False:
            #sendbuf = []
            sendbuf.clear()
            if (len(txBuf) == 0 or rxLen <= 3):
                rxbuf = []
                return rxbuf

            for i in range(len(txBuf)):
                if txBuf[i] < 0 or 255 < txBuf[i]:
                    rxbuf = []
                    return rxbuf
                sendbuf.append(txBuf[i])

            self.__isSynchronize = True

            # print('sendData-->',sendbuf)

            
            self.com.flushInput()
            self.STMKondoWrite(sendbuf, len(sendbuf), rxLen)
            #self.com.write(sendbuf)
            #if not writeOnly:
            rxbuf = self.com.read(rxLen)
            print(f"Sent      {bytearray(sendbuf)}")
            print(f"Recieved: {bytearray(rxbuf)}")
            self.com.flushInput()
            if rxbuf is not None and len(rxbuf) != 0:
                #if len(rxbuf) == rxLen and rxbuf[0] == rxLen:
                if self.__checkCheckSum(rxbuf) == False:
                    rxbuf = []
                #else:
                #rxbuf = []
            else:
                rxbuf = []
            #else:
             #   self.com.flushInput()
              #  rxbuf = []
            self.__isSynchronize = False
            # print('readData-->',rxbuf)
            return rxbuf
        else:
            rxbuf = []
            return rxbuf

    @staticmethod
    def moveComToRamCmd(destAddr, destData):
        txbuf = []
        if len(destData) > 249:
            return 0, Rcb4BaseLib.CmdOkType.Error
        txbuf.append((len(destData) + 7) & 0xff)
        txbuf.append(Rcb4BaseLib.CommandTypes.Move)
        txbuf.append(Rcb4BaseLib.SubMoveCmd.ComToRam)
        txbuf.append(destAddr & 0xff)
        txbuf.append((destAddr >> 8) & 0xff)
        txbuf.append(0x00)

        if len(destData) == 1:
            txbuf.append(destData[0])
        else:
            for i in range(len(destData)):
                txbuf.append(destData[i])

        txbuf.append(Rcb4BaseLib.CheckSum(txbuf))

        returnDataSize = 4

        return returnDataSize, txbuf

    def moveComToRamCmdSynchronize(self, scrAddr, destData):

        readSize, sendData = self.moveComToRamCmd(scrAddr, destData)
        if readSize > 0:
            rxbuf = self.synchronize(sendData, readSize)
            if len(rxbuf) > 3 and Rcb4BaseLib.AckType.Ack == rxbuf[2]:
                return True
            else:
                return False
        else:
            return False

    @staticmethod
    def callCmd(romAddr):
        buf = []
        buf.append(0x07)
        buf.append(Rcb4BaseLib.CommandTypes.Call)
        buf.append(romAddr & 0xff)
        buf.append((romAddr >> 8) & 0xff)
        buf.append((romAddr >> 16) & 0xff)
        buf.append(0x00)
        buf.append(Rcb4BaseLib.CheckSum(buf))
        return 4, buf

    def synchronizeAck(self, txData):
        rxbuf = self.synchronize(txData, 4)
        if len(rxbuf) > 3 and self.AckType.Ack == rxbuf[2]:
            return True
        else:
            return False

    @staticmethod
    def acknowledgeCmd():
        txbuf = [0x04, Rcb4BaseLib.CommandTypes.AckCheck, Rcb4BaseLib.AckType.Ack, 0]
        txbuf[3] = Rcb4BaseLib.CheckSum(txbuf)

        returnDataSize = 4

        return returnDataSize, txbuf

    @staticmethod
    def moveRamToComCmd(scrAddr, scrDataSize):
        txbuf = []
        txbuf.append(0x0a)
        txbuf.append(Rcb4BaseLib.CommandTypes.Move)
        txbuf.append(Rcb4BaseLib.SubMoveCmd.RamToCom)
        txbuf.append(0x00)
        txbuf.append(0x00)
        txbuf.append(0x00)
        txbuf.append(scrAddr & 0xff)
        txbuf.append((scrAddr >> 8) & 0xff)
        txbuf.append(scrDataSize)
        txbuf.append(Rcb4BaseLib.CheckSum(txbuf))

        returnDataSize = scrDataSize + 3

        return returnDataSize, txbuf

    def moveRamToComCmdSynchronize(self, scrAddr, scrDataSize):
        readSize, sendData = self.moveRamToComCmd(scrAddr, scrDataSize)
        if readSize > 0:
            rxbuf = self.synchronize(sendData, readSize)
            if len(rxbuf) < readSize:
                rxbuf = []
                return False, rxbuf

            destData = []
            if scrDataSize == 1:
                destData.append(rxbuf[2])

            else:
                for i in range(scrDataSize):
                    destData.append(rxbuf[i + 2])

            return True, bytes(destData)

        else:
            rxbuf = []
            return False, rxbuf


    def moveComToDeviceCmd (self, icsNum, offset, destData):

        buf =[]
        destSize = len(destData)
        if((icsNum < 0)or(self.IcsDeviceSize < icsNum)or (offset < 0) or(self.IcsDeviceDataSize  <(offset+destSize))):
            return -1,buf

        buf.append (0x07 + len(destData))
        buf.append (self.CommandTypes.Move)
        buf.append (self.SubMoveCmd.ComToDevice)
        buf.append (offset)
        buf.append (icsNum)
        buf.append (0x00)
        if len(destData) == 1:
             buf.append (destData[0])
        else:
            for i in range(len(destData)):
                buf.append (destData[ i ])
        buf.append (Rcb4BaseLib.CheckSum(buf))
        return 4,buf

    def moveComToDeviceCmdSynchronize(self,icsNum, offset, destData):
        readSize, sendData = self.moveComToDeviceCmd (icsNum, offset, destData)
        if readSize > 0:
            rxbuf = self.synchronize(sendData, readSize)
            if len(rxbuf) > 3 and self.AckType.Ack ==rxbuf[2]:
                return True
            else:
                return False
        else:
            return False


    def getConfig(self):
        retf, rxbuf = self.moveRamToComCmdSynchronize(Rcb4BaseLib.RamAddr.ConfigRamAddress, 2)

        if (retf == False) or (len(rxbuf) != 2):
            return 0xFFFF
        else:
            return (rxbuf[1] * 256 + rxbuf[0])

    def checkAcknowledge(self):
        reSize, txbuf = self.acknowledgeCmd()
        rxbuf = self.synchronize(txbuf, reSize)
        if len(rxbuf) == 0:
            return False
        else:
            return True

    def getMotionPlayNum(self):

        retf, retbuf = self.moveRamToComCmdSynchronize(self.RamAddr.ProgramCounterRamAddress, 10)

        if (retf == False) or (len(retbuf) != 10):
            return -1

        eflfg = retbuf[3] + retbuf[4] + retbuf[5] + retbuf[6] + retbuf[7]

        pcunt = retbuf[0] + (retbuf[1] << 8) + (retbuf[2] << 16)

        mno = int(((pcunt - Rcb4BaseLib.RomAddr.MotionRomAddress) / self.MotionSingleDataCount) + 1)

        if eflfg == 0:
            return 0
        if pcunt < Rcb4BaseLib.RomAddr.MotionRomAddress:
            return 0
        if mno < 0 or mno > 120:
            return -2

        return mno

    def suspend(self):

        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableRunEeprom
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableServoResponse
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableReferenceTable
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableSio

        txbuf = [self.__configData & 0xff, (self.__configData >> 8) & 0xff]
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.ConfigRamAddress, txbuf)

    def motionAddr2motionNum(self, motionNum):
        if 0 < motionNum <= self.maxMotionCount:
            return (motionNum - 1) * self.MotionSingleDataCount + Rcb4BaseLib.RomAddr.MotionRomAddress
        else:
            return -1

    def setMotionNum(self, motionNum):
        if 0 < motionNum <= self.maxMotionCount:
            _, buf = self.callCmd(self.motionAddr2motionNum(motionNum))
            return self.synchronizeAck(buf)
        else:
            return False

    def resetProgramCounter(self):
        buf = [Rcb4BaseLib.RomAddr.MainLoopCmd & 0xff, (Rcb4BaseLib.RomAddr.MainLoopCmd >> 8) & 0xff, 0, 0,
               0, 0, 0, 0, 0, 0]
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.ProgramCounterRamAddress, buf)

    def resume(self):
        self.__configData |= Rcb4BaseLib.ConfigData.EnableRunEeprom
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableServoResponse
        self.__configData |= Rcb4BaseLib.ConfigData.EnableReferenceTable
        self.__configData |= Rcb4BaseLib.ConfigData.EnableSio
        buf = [self.__configData & 0xff, (self.__configData >> 8) & 0xff]
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.ConfigRamAddress, buf)


    def userParameterAddr(self, ParameterNum):
        if 0 < ParameterNum <= self.UserParameterCount:
            return Rcb4BaseLib.RamAddr.UserParameterRamAddress + (
                        (ParameterNum - 1) * self.UserParameterSingleDataCount)
        else:
            return -1

    @staticmethod
    def setServoNo (servoDatas):
      ret = 0

      for idat in servoDatas:
        no = Rcb4BaseLib.icsNum2id(idat.id, idat.sio)
        sf = 0x1
        ret |=  (sf << no)
      return ret << 24

    def userCounterAddr(self, counterNum):
        if (0 < counterNum <= self.CounterCount):
            return self.RamAddr.CounterRamAddress + ((counterNum - 1) * self.CounterSingleDataCount)
        else:
            return -1

    @staticmethod
    def runConstFrameServoCmd (servoDatas,frame):
        buf =[]

        sDatas = []
        if type(servoDatas) == Rcb4BaseLib.ServoData:
            sDatas.append(servoDatas)
        else:
            sDatas = servoDatas

        if Rcb4BaseLib().checkServoDatas(sDatas) == False:
            return -1,buf

        wk = Rcb4BaseLib.setServoNo (sDatas) >> 24
        buf.append (len(sDatas) * 2 + 9)
        buf.append ( Rcb4BaseLib.CommandTypes.ConstFrameServo)
        buf.append (wk & 0xff)
        buf.append ((wk >>  8) & 0xff)
        buf.append ((wk >> 16) & 0xff)
        buf.append ((wk >> 24) & 0xff)
        buf.append ((wk >> 32) & 0xff)
        buf.append (frame)
        servoDatas = sorted(sDatas)
        for idat in servoDatas:
          buf.append(idat.data & 0xff)
          buf.append((idat.data >> 8)& 0xff)
        buf.append(Rcb4BaseLib.CheckSum(buf))
        return 4,buf

    @staticmethod
    def runKondoMVFrameServoCmd (kondoMVServoDatas,frame):
        """Specific function for fast working sending

        Args:
            kondoServoDatas (list of servoData): ordered list. [servoData[id=1, sio=1], 
                                                                servoData[id=1, sio=2], 
                                                                servoData[id=2, sio=1],
                                                                servoData[id=2, sio=2],
                                                                ...
                                                                ]
            frame ([type]): same as runConstFrameServoCmd()

        Returns:
            [type]: same as runConstFrameServoCmd()
        """
        buf = [
            0x37,              # message size
            Rcb4BaseLib.CommandTypes.ConstFrameServo,  # set servo pos command
            0xff,                                      # 1 to 4 servos in both sio (8 servos)
            0xff,                                      # 5 to 8 servos in both sio (8 servos)
            0x7f,                                      # 9 to 12 servos in both sio (8 servos)
            0x0,               # 13 to 16 servos in both sio
            0x0,                 # 13 to 16 servos in both sio
            frame
            ]

        sDatas = kondoMVServoDatas

        if Rcb4BaseLib().checkServoDatas(sDatas) == False:
            return -1,buf

        for idat in sDatas:
          buf.append(idat.data & 0xff)
          buf.append((idat.data >> 8)& 0xff)
        buf.append(Rcb4BaseLib.CheckSum(buf))
        return 4,buf

    def checkServoDatas(self, servoDatas):
        checkData = []

        for sData in servoDatas:
            if not (type(sData) is Rcb4BaseLib.ServoData):
                return False
            cData = sData.icsNum2id()
            if not (0 <= cData  <= self.IcsDeviceSize):
                return False

            checkData.append(cData)

        if len(servoDatas) == len(set(checkData)):
            return True
        else:
            return False

    @staticmethod
    def checkSio(sio):
        return sio == 0x01 or sio == 0x02

    @staticmethod
    def icsNum2id(id, sio):
        return id * 2 + (sio - 1)

    @staticmethod
    def runSingleServoCmd(idNum, sioNum, pos, frame):
        buf = []
        buf.append(0x07)
        buf.append(Rcb4BaseLib.CommandTypes.SingleServo)
        buf.append(Rcb4BaseLib.icsNum2id(idNum, sioNum))
        buf.append(frame)
        buf.append(pos & 0xff)
        buf.append((pos >> 8) & 0xff)
        buf.append(Rcb4BaseLib.CheckSum(buf))
        return 4, buf


    def moveDeviceToComCmd(self, icsNum, offset, dataSize):
        buf =[]

        if((icsNum < 0)or(self.IcsDeviceSize < icsNum)or (offset < 0) or(self.IcsDeviceDataSize  <(offset+dataSize))):
            return -1,buf

        buf.append (0x0a)
        buf.append ( self.CommandTypes.Move)
        buf.append ( self.SubMoveCmd.DeviceToCom)
        buf.append ( 0x00)
        buf.append ( 0x00)
        buf.append ( 0x00)
        buf.append ( offset)
        buf.append ( icsNum)
        buf.append (dataSize)
        buf.append (Rcb4BaseLib.CheckSum(buf))

        returnDataSize = dataSize+3

        return returnDataSize,buf

    def moveDeviceToComCmdSynchronize(self,icsNum, offset, dataSize):

        readSize, sendData  = self.moveDeviceToComCmd(icsNum, offset, dataSize)

        if readSize > 0:
            rxbuf = self.synchronize(sendData, readSize)

            if len(rxbuf)< readSize:
                rxbuf = []
                return False,rxbuf

            destData = []
            if dataSize == 1:
                destData.append (rxbuf[2])

            else:
                for i in range(dataSize):
                    destData.append (rxbuf[i+2])

            return True, bytes(destData)

    def adDataAddr(self, adPort):
        if adPort >= 0 and adPort < 11:
            return self.RamAddr.AdcRamAddress + adPort * 2
        else:
            return  -1

    def setParametersBaseCmd(self, servoDatas, servoParameter):
        buf = []

        sDatas = []
        if type(servoDatas) == self.ServoData:
            sDatas.append(servoDatas)
        else:
            sDatas = servoDatas

        if self.checkServoDatas(sDatas) == False:
            return -1, buf

        wk = self.setServoNo(sDatas) >> 24
        buf.append(len(sDatas) + 9)
        buf.append(self.CommandTypes.ServoParam)
        buf.append(wk & 0xff)
        buf.append((wk >> 8) & 0xff)
        buf.append((wk >> 16) & 0xff)
        buf.append((wk >> 24) & 0xff)
        buf.append((wk >> 32) & 0xff)
        buf.append(servoParameter)
        servoDatas = sorted(sDatas)
        for idat in sDatas:
            if 0 < idat.data < 128:
                buf.append(idat.data & 0xff)
            else:
                buf = []
                return 0, buf
        buf.append(self.CheckSum(buf))
        return 4, buf

    def setSpeedCmd(self, servoDatas):
        return self.setParametersBaseCmd(servoDatas, 2)

    def setStretchCmd(self, servoDatas):
        return self.setParametersBaseCmd(servoDatas, 1)


    #runs uploaded motion
    def motionPlay(self, motionNum):

        if (motionNum <= 0) or (self.maxMotionCount < motionNum):
            return False

        if self.suspend() == False:
            return False

        if self.resetProgramCounter() == False:
            return False

        if self.setMotionNum(motionNum) == False:
            return False

        return self.resume()


    #sets U constants
    def setUserParameter(self, ParameterNum, data):
        addr = self.userParameterAddr(ParameterNum)
        try:
            buf = struct.pack('<h', data)
        except:
            return False

        if not (addr < 0):
            return self.moveComToRamCmdSynchronize(addr, buf)
        else:
            return False



    #gets U constants
    def getUserParameter(self, ParameterNum):
        addr = self.userParameterAddr(ParameterNum)
        if not (addr < 0):
            retf, retbuf = self.moveRamToComCmdSynchronize(addr, 2)
            if (retf == True) and (len(retbuf) == 2):
                paraData = struct.unpack('<h', retbuf)[0]
                return True, paraData
            else:
                return False, -1
        else:
            return False, -1

    def __del__(self):
        self.close()


    #sets C user counter
    def setUserCounter(self,counterNum,data):

        addr = self.userCounterAddr(counterNum)
        if (addr < 0):
            return False

        try:
            sendData = struct.pack('B',data)
        except:
            return False

        return self.moveComToRamCmdSynchronize(addr, sendData)

    #gets C user counter
    def getUserCounter(self,counterNum):
        addr = self.userCounterAddr(counterNum)
        if(addr < 0):
            return False,-1
        else:
            retf, retbuf = self.moveRamToComCmdSynchronize(addr , 1 )

            if(retf == False) or (len(retbuf) != 1):
                return False,-1
            else:
                countData  =  struct.unpack_from('B',retbuf,0)[0]
                return True, countData


    #moves single servo
    def setSingleServo(self, id, sio, pos, frame):
        if not Rcb4BaseLib.checkSio(sio):
            print('Wrong Sio. 1 / 2 for left / right side')
            return False
        _, txbuf = self.runSingleServoCmd(id, sio, pos, frame)
        if len(txbuf) == 0:
            return False
        return self.synchronizeAck(txbuf)


    #gets position of single servo
    def getSinglePos(self,id,sio):
        if not Rcb4BaseLib.checkSio(sio):
            return False, -1
        retf,retbuf = self.moveDeviceToComCmdSynchronize(self.icsNum2id(id, sio), Rcb4BaseLib.DeviceAddrOffset.MotorPositionAddressOffset, 2)

        if(retf == True) and (len(retbuf) ==2):
            posData  =  struct.unpack('<h',retbuf)[0]
            return  True, posData

        else:
            return False, -1

    #gets trim of single servo
    def getSingleTrim(self,id,sio):
        if not Rcb4BaseLib.checkSio(sio):
            print('Wrong Sio. 1 for left, 2 for right side')
            return False, -1
        retf,retbuf = self.moveDeviceToComCmdSynchronize(self.icsNum2id(id, sio), Rcb4BaseLib.DeviceAddrOffset.TrimAddressOffset, 2)

        if(retf == True) and (len(retbuf) ==2):
            trimData  =  struct.unpack('<h',retbuf)[0]
            return  True, trimData

        else:
            return False, -1

    #test Ram trim offset
    def setSingleTrim(self,id ,sio, trim):
        if not Rcb4BaseLib.checkSio(sio):
            print('Wrong Sio. 1 for left, 2 for right side')
            return False, -1
        buf =[trim]
        offset = Rcb4BaseLib.DeviceAddrOffset.TrimAddressOffset
        reFlag = self.moveComToDeviceCmdSynchronize(self.icsNum2id(id, sio), offset, buf)
        return reFlag


    # get analog port data
    def getAdData(self,adPort):
        if adPort >= 0 and adPort < 11:
            retf,rxbuf = self.moveRamToComCmdSynchronize(self.adDataAddr(adPort), 2 )
            if (retf == True) and (len(rxbuf) == 2):
                return rxbuf[1] * 256 + rxbuf[0]

        return 0xffff

    # get analog data from all ports
    def getAllAdData(self):
        retf,retbuf = self.moveRamToComCmdSynchronize(Rcb4BaseLib.RamAddr.AdcRamAddress  ,self.AdcDataCount)
        redate = []
        if (retf == True) and (len(retbuf) == (self.AdcCount * 2)):
            for i in range(self.AdcCount):
                redate.append( retbuf[1+ i*2] * 256 + retbuf[0+ i*2])
        return retf,redate

    # get digital port data
    def getPio(self):
        retf,rxbuf = self.moveRamToComCmdSynchronize(self.RamAddr.PioAddress ,2)
        if(retf == True) and (len(rxbuf) == 2):
            return ((rxbuf[1] & 0x03) * 256) + rxbuf[0]
        else:
            return -1

    # set digital port data
    def setPio (self, pioData):
        buf=[pioData & 0xff,(pioData >> 8) & 0x03]
        return self.moveComToRamCmdSynchronize(self.RamAddr.PioAddress, buf)

    # change digital port mode (input is 0 or output is 1)
    def setPioMode (self, pioModeData):
        buf=[pioModeData & 0xff,(pioModeData >> 8) & 0x03]
        return self.moveComToRamCmdSynchronize(self.RamAddr.PioModeAddres, buf)


    # sets potitions of several servos using ServoData class
    def setServoPos (self,servoDatas,frame):
        _,txbuf = self.runConstFrameServoCmd(servoDatas,frame)
        return  self.synchronizeAck(txbuf)


    # sets potitions of several servos using ServoData class
    def setKondoMVServoPos (self,servoDatas,frame):
        rxLen ,txbuf = self.runKondoMVFrameServoCmd(servoDatas,frame)
        return self.synchronize(txbuf, rxLen, writeOnly=True)

    def setServoSpeed(self, servoDatas):
        _, txbuf = self.setSpeedCmd(servoDatas)
        return self.synchronizeAck(txbuf)

    def setServoStretch(self, servoDatas):
        _, txbuf = self.setStretchCmd(servoDatas)
        return self.synchronizeAck(txbuf)

    





if __name__ == "__main__":
    
    wtf=bytearray(b"\x09\x00\x02\x00\x00\x00\xA0\x42\xED")
    
    deinitSTM()
    time.sleep(2)
    initSTM()
    print("Starting STM32...")
    time.sleep(3)
    
    kondo = Rcb4BaseLib()
    kondo.open('/dev/ttyAMA2',1250000, 1.3) #1250000
    
    sd = kondo.ServoData()
    sd.id   = 8
    sd.sio  = 1 
    
    x=0
    while True:
        sd.data = 7500 + int(500 * sin(0.1*x))
        kondo.setServoPos([sd], 5)
        x+=1
        time.sleep(0.05)
    




