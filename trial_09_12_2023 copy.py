import logging
import sys
import time
import datetime
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import pandas as pd

#Setting URI address:
uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E710')

logging.basicConfig(level=logging.ERROR)

#initializing python dictionaries to hold different values of interest (will be exported to xls or csv)
stabilizer_dict = {"stabilizer.roll":[], "stabilizer.pitch":[], "stabilizer.yaw":[]}
state_estimate_dict = {"stateEstimate.x":[],"stateEstimate.y":[],"stateEstimate.z":[]}
range_dict = {"range.front":[],"range.back":[],"range.left":[],"range.right":[],"range.up":[]}

#making list for easy access
logging_dicts = [stabilizer_dict, state_estimate_dict, range_dict]

#Drone class will contain functions related to movement of drone
class Drone():
    def __init__(self, scf, pc):
        self.pc = pc
        self.scf = scf
        
    def rise(self, distance):
        self.pc.up(distance)

    def land(self):
        self.pc.stop()
        return
    
    def forward(self, distance):
        self.pc.forward(distance)
        return
    def backward(self,distance):
        self.pc.back(distance)
        return
    def left(self, angle=90):
        self.pc.turn_left(angle_degrees=angle)
        return
    def right(self, angle=90):
        self.pc.turn_right(angle_degrees=angle)
        return
    
    #TODO 
    def is_Wall(self):
        #determine if obstacle in front is the wall or something to go around
        return False
    
    #TODO
    def go_Around(self, path: str):
        #turn right 
        #check if obstacle in front if so break so that method is recalled
        #go forward until nothing on the left side
        #turn left
        #go forward until nothing on left side
        #turn left
        #go forward same amount as first iteration
        #turn right
        return

    #TODO 
    def search_Land(self, distance=2, found = False, at_l_wall = False, at_r_wall = False):
        #assuming we start in the right corner of the section: -------------
                                                   #                      * |
                                                   #                        |       
                                                   #
        #while the landing area is not found, the drone will move left, then back, then right, then back .
        # This will repeat until landing pad is found
        #                        
        while not found:
            while not found and not at_l_wall:
                if range_dict["range.left"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around()
                        continue
                    else:
                        at_l_wall = True
                        break

                self.pc.left(.2)

                if range_dict['range.up'][-1] < distance:
                    found = True
                    break

            if not found:
                if range_dict['range.back'][-1] < distance:
                    if not self.is_Wall():
                        self.go_Around('back')
                self.pc.back()
                        
    

            while not found and not at_r_wall:
                if range_dict["range.right"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around('right')
                        continue
                    else:
                        at_r_wall = True
                        break

                self.pc.right(.2)
                if range_dict['range.up'][-1] < distance:
                    found = True
                    break
                
            
            if not found:
                if range_dict['range.back'][-1] < distance:
                    if not self.is_Wall():
                        self.go_Around('back')
                self.pc.back(.2)

        
        

        #search for landing box using _____ method
        #break when find box
        return
    
    #TODO     
    def find_center(self):
        #at end of the box
        #get that location
        #find other edge of the box
        #find length of box and go to center of it
        #repeat for other direction
        return
    
    #TODO
    def go_to(self,distance):
        #travels the distance needed avoiding obstacles
        return

    #TODO 
    def search_Start(self):
        #at assumed start position
        #if not above box
        #search for box using growing circles
        #break when find box
        return
    


    def log_async(self, logconf):
        cf = self.scf.cf
        for log in logconf:
            cf.log.add_config(log)
            log.data_received_cb.add_callback(self.log_callback)
            log.start()
            log.stop()

    def log_callback(self, timestamp, data, logconf):
        #for log_dict in logging_dicts: #look at each dictionary for each value of interest
        if logconf.name == 'Stabilizer':
            for param in logging_dicts[0]: #look at each item in the dictionary
                logging_dicts[0][param].append(data[param]) #add the value associated with that item to the list for that item
        elif logconf.name == 'Range':
            for param in logging_dicts[2]: #look at each item in the dictionary
                logging_dicts[2][param].append(data[param]) #add the value associated with that item to the list for that item
        elif logconf.name == 'Position':
            for param in logging_dicts[1]: #look at each item in the dictionary
                logging_dicts[1][param].append(data[param]) #add the value associated with that item to the list for that item

        # for k in data:
        #     data_csv[k].append(data[k])
        #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        #print(type(data))

#initialization required for asyn logger (look at documentation for this)
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
    #initialization of drivers and logger (see documentation)
    cflib.crtp.init_drivers()
    logs = init_logs()
    #for movement simplicity everything should be right angles
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf, default_height= .5) as pc:
            drone = Drone(scf, pc)
            #initial_pos = [state_estimate_dict['stateEstimate.x'][0],state_estimate_dict['stateEstimate.y'[0]]]
            drone.log_async(logs)
            drone.rise(.5)
            for i in range(10):
                drone.forward(.2)
                if range_dict["range.front"][-1]<500:
                    drone.backward(.2)
                    print("STOP")
                    break
            #drone.takeoff(0.1) 
            #time.sleep(1) ##hovering for 3 seconds
            #drone.forward(0.3)
            #drone.sleep(3)
            #drone.land()

        #   TODO
        #   Fly code 
    #         at_end = False
    #         while not at_end:
    #             if range_dict["range.front"][-1]<500:
    #                 if drone.is_wall():
    #                     at_end=True
    #                     break
    #                 drone.go_Around()
    #                 continue
    #             drone.forward(.2)
    #         drone.search_Land()
    #         drone.find_center()
    #         end_pos= [state_estimate_dict['stateEstimate.x'][-1],state_estimate_dict['stateEstimate.y'[-1]]]
    # with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    #     with MotionCommander(scf, default_height= .8) as pc:
    #         drone = Drone(scf,pc)
    #         distance=[initial_pos[0]-end_pos[0],initial_pos[1]-end_pos[1]]
    #         drone.return_Start(distance)
    #         drone.search_Start()
    #         drone.find_center()
    

    
    #creating pandas dataframes from logging dicts
    stabilizer_df = pd.DataFrame(stabilizer_dict)
    state_estimate_df = pd.DataFrame(state_estimate_dict)
    range_df = pd.DataFrame(range_dict)

    #writing each df to xlsx file
    x = str(datetime.datetime.now().replace(microsecond=0)).replace(":","")
    path = "./data/test_" + x +".xlsx"
    with pd.ExcelWriter(path= path, engine='xlsxwriter') as writer:
        for index, df in enumerate([stabilizer_df, state_estimate_df, range_df], start=0):
            df.to_excel(writer,sheet_name= list(logging_dicts[index].keys())[0].split(".")[0])
            #df.to_excel(writer,sheet_name= str(index))
        # drone1.hover(1)
        # drone1.land()

