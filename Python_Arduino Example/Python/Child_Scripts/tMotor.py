import numpy as np
import serial
import time


class TMOTOR(object):
    def __init__(self, ComPort) -> None:
       #Motor Variables -------
       self.ComPort = ComPort
       self.temperature_int16 = 0
       self.temperature = 0
       self.torque_int16 = 0
       self.torque = 0
       self.speed_int16 = 0
       self.speed = 0
       self.position_int16 = 0
       self.position = 0
       self.torque_sent_int16 = 0xffff
       self.torque_sent = 0
       self.torque_sent_highbyte = 0xffff
       self.torque_sent_lowbyte = 0xffff
       self.Max_Temp = 0
       self.Max_Torque = 0
       self.Max_Speed = 0
       self.Peak_Torque = 0
       self.L_CMD_int_b1 = 0xffff
       
       #Serial Variables -------
       self.buffer = 0x00
       self.buffer_len = 0x00

       #Serial Begin -------------
       #self.Serial_IMU = serial.Serial(ComPort, 230400, timeout=0.007, parity=serial.PARITY_NONE)
       self.Serial_Motor = serial.Serial(ComPort, 115200, timeout=0.01, parity=serial.PARITY_NONE)
        
       print('Serial Open Success')
       #Serial END---------------

    def ToUint(self, x, x_min, x_max, nbits):
        span = x_max - x_min

        if (x < x_min):
            x = x_min

        if (x > x_max):
            x = x_max
        toUint=((x - x_min) * ((float)((1 << nbits) - 1) / span))
        return toUint
        

    def ToFloat(self,x_int, x_min, x_max, nbits):
        span = x_max - x_min
        offset_value = x_min
        toFloat= x_int * span / float((((1 << nbits) - 1))) + offset_value
        return toFloat

    def read(self):
        self.buffer = self.Serial_Motor.read(11)
        self.buffer_len = len(self.buffer)

    def decode(self):
        if len(self.buffer)==11 and self.buffer[0] == 0xaa and self.buffer[1] == 0xbb :  
            if self.buffer[4]==0xcc:
                if self.buffer[7]==0xdd:
                    self.torque_int16 =(self.buffer[2] <<8) | (self.buffer[3])
                    self.torque = self.ToFloat(self.torque_int16,-10, 10,16)
                    print("Torque: " + str(round(self.torque, 2)))
        
                    self.speed_int16 =(self.buffer[5] <<8 | (self.buffer[6]))
                    self.speed = self.ToFloat(self.speed_int16, -25, 25, 16)
                    print("Speed: " + str(round(self.speed, 2)))
        
                    self.position_int16 =(self.buffer[8] <<8 | (self.buffer[9]))
                    self.position = self.ToFloat(self.position_int16, 0,360,16)                  
                    print("Position: " + str(round(self.position, 2)))
                    print()
        else : 
            print("Decode Failure") 
            print()

    def sendTorque(self, torque_sent): 
        self.torque_sent_int16 = self.ToUint(torque_sent, -10, 10,16)
        self.torque_sent_highbyte = np.uint8(np.uint16(self.torque_sent_int16) >> 8)
        self.torque_sent_lowbyte = np.uint8(np.uint16(self.torque_sent_int16) & 0xff)
        self.Serial_Motor.write(bytes([0xff, 0xee, 0xdd, self.torque_sent_lowbyte, self.torque_sent_highbyte, 0xbb]))
        print("Torque Sent: " + str(torque_sent))
        
        #self.Serial_Motor.write(bytes([0xff , 0xee , 0xdd , 0x77, 0x77, 0xbb])