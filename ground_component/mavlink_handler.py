import sys, os
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
import threading
#from pymavlink.dialects.v20.copperstone import *

class MavlinkHandler():
    def __init__(self, connection_string, cb = None):
        mavutil.set_dialect("ardupilotmega")
        self.m = mavutil.mavlink_connection(connection_string)
        self.cb = cb
        self.running = True
        read_th = threading.Thread(target=self.read_thread)
        read_th.start()
    
    def shutdown(self):
        self.running = False

    def read_thread(self):
        while (self.running):
            msg = self.m.recv_match(blocking=True, timeout=1)
            if not msg:
                return
            if msg.get_type() == "BAD_DATA":
                print("bad data")
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()
            else:
                if self.cb:
                    self.cb(msg)
