import time
from tcp_interface import TCP_Interface
from pymavlink.dialects.v10.ardupilotmega import *

import base64
import binascii



class Mavlink_Interface:
    def __init__(self, tcp_link):
        self.tcp_link = tcp_link
        self.tcp_link.bindDataCallback(self.mavlinkMessageParser)
        self.mav_object = MAVLink(None, 0, 255, use_native=False)
        self.data= bytearray()

    def gotMavlinkMessage(self, msg):
       #print(msg)
       pass
       
    def mavlinkMessageParser(self, b):
        self.data.append(b[0])
        if (b == b'='):
            try:
                payload = bytearray(base64.b64decode(self.data))
                self.gotMavlinkMessage(self.mav_object.decode(payload))
            except binascii.Error:
                print("Mavlink payload is has invalid encoding")
            except MAVError:
                print("Invalid MAVLINK message")
            self.data = bytearray()

    def serialize(self, mavlink_message):
        return base64.b64encode(mavlink_message.pack(self.mav_object))
    
    def sendMavlinkMessage(self, msg):
        dataTemp = self.serialize(msg)
        self.tcp_link.write(dataTemp)


tcp_link = TCP_Interface('localhost', 65123)

mav_int = Mavlink_Interface(tcp_link)

print("started")
i = 0
while True:
    msg = MAVLink_heartbeat_message(0,0,0,i,0,0)
    mav_int.sendMavlinkMessage(msg)
    time.sleep(1)
    i = i +1


