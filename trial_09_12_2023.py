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
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E710')
logging.basicConfig(level=logging.ERROR)

class Drone():
    def __init__(self, scf, mc):
        self.mc = mc
        self.scf = scf
        

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
    
    def log_async(self, logconf):
        cf = self.scf.cf
        for log in logconf:
            cf.log.add_config(log)
            log.data_received_cb.add_callback(self.log_callback)
            log.start()
            log.stop()

    def log_callback(self, timestamp, data, logconf):
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))


def init_logs():
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_range = LogConfig(name='Range', period_in_ms=10)
    lg_range.add_variable('range.front','uint16_t')
    lg_range.add_variable('range.back','uint16_t')
    lg_range.add_variable('range.left','uint16_t')
    lg_range.add_variable('range.right','uint16_t')
    lg_range.add_variable('range.up','uint16_t')
    lg_pos = LogConfig(name='Position', period_in_ms=10)
    lg_pos.add_variable('stateEstimate.x','float')
    lg_pos.add_variable('stateEstimate.y','float')
    lg_pos.add_variable('stateEstimate.z','float')
    return [lg_stab,lg_range,lg_pos]


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    logs = init_logs()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf, default_height= 0.3) as mc:
            drone = Drone(scf, mc)
            drone.log_async(logs)
            #drone.takeoff(0.1) 
            time.sleep(1) ##hovering for 3 seconds
            #drone.forward(0.3)
            #drone.sleep(3)
            drone.land()
        # drone1.hover(1)
        # drone1.land()
