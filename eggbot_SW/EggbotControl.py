# -*- coding: utf-8 -*-
"""
Created on Fri Apr 22 01:01:43 2022

@author: lakij
"""

import serial
import numpy
import struct

class EggbotControl:
    
    def __init__(self, port):
        self.port = port;
        self.ser = serial.Serial(port, 115200, timeout = 0.1);
        
    def lift_servo(self):
        self.ser.write(b'pu')
        self.ser.timeout = 0.1
        resp = self.ser.read(1)
        if resp != b'a':
            raise Exception("Unexpected response",resp)
        resp = self.ser.read(2)
        if resp != b'pu':
            raise Exception("Unexpected response",resp)
            
    def lower_servo(self):
        self.ser.write(b'pd')
        self.ser.timeout = 1
        resp = self.ser.read(1)
        if resp != b'a':
            raise Exception("Unexpected response",resp)
        resp = self.ser.read(2)
        if resp != b'pd':
            raise Exception("Unexpected response",resp)
            
    def zero_steppers(self):
        self.ser.write(b'z')
        self.ser.timeout = 60
        resp = self.ser.read(1)
        if resp != b'a':
            raise Exception("Unexpected response",resp)
        resp = self.ser.read(1)
        if resp != b'z':
            raise Exception("Unexpected response",resp)
            
    def set_speed(self, x_speed, y_speed):
        self.ser.write(b's')
        x_speed = int(x_speed)
        y_speed = int(y_speed)
        self.ser.write(struct.pack("H",int(x_speed)))
        self.ser.write(struct.pack("H",int(y_speed)))
        self.ser.timeout = 0.1
        resp = self.ser.read(1)
        if resp != b'a':
            raise Exception("Unexpected response",resp)
        resp = self.ser.read(1)
        if resp != b's':
            raise Exception("Unexpected response",resp)
        resp = self.ser.read(4)
        if resp == b'\xFF\xFF\xFF\xFF':
            raise Exception("Error setting speed")
            
    def move(self, x_rel, y_rel):
        self.ser.write(b'm')
        x_rel = int(x_rel)
        y_rel = int(y_rel)
        self.ser.write(struct.pack("h",int(x_rel)))
        self.ser.write(struct.pack("h",int(y_rel)))
        self.ser.timeout = 60
        resp = self.ser.read(1)
        if resp != b'a':
            raise Exception("Unexpected response",resp)
        self.ser.timeout = 0.1
        resp = self.ser.read(1)
        if resp != b'm':
            raise Exception("Unexpected response",resp)
        resp = self.ser.read(4)
        if resp == b'\xFF\xFF\xFF\xFF':
            raise Exception("Error issuing move command")
            
    def get_position(self):
        self.ser.write(b'ra')
        self.ser.timeout = 0.1
        resp = self.ser.read(7)
        if resp[:3] != b'ara':
            raise Exception("Unexpected response",resp)
        x_pos = struct.unpack('h',resp[3:5])[0]
        y_pos = struct.unpack('h',resp[5:7])[0]
        return x_pos, y_pos
        
    def get_max_position(self):
        self.ser.write(b'rm')
        self.ser.timeout = 0.1
        resp = self.ser.read(7)
        if resp[:3] != b'arm':
            raise Exception("Unexpected response",resp)
        x_pos = struct.unpack('h',resp[3:5])[0]
        y_pos = struct.unpack('h',resp[5:7])[0]
        return x_pos, y_pos
        
if __name__ == "__main__":
    
    port = "COM7"
    ctrl = EggbotControl(port)
    
            
        