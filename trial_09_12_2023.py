import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander


uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E10')

class Drone():
    def __init__(self, scf):
        self.mc = MotionCommander(scf, default_height=0.3)
        

    def rise(self, distance):
        self.mc.up(distance)

    def land(self):
        self.mc.stop()
        return
    
    def forward(self, distance):
        self.mc.forward(distance)
        return
    def left(self, angle):
        self.mc.turn_left(angle_degrees=angle)
        return
    def right(self, angle):
        self.mc.turn_right(angle_degrees=angle)
        return


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    drone = Drone()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf, default_height= 0.1):
            drone.takeoff(0.5) 
            time.sleep(3) ##hovering for 3 seconds
            drone.forward(0.3)
            drone.sleep(3)
            drone.land()

        # drone1.hover(1)
        # drone1.land()

