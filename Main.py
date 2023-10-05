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
from cflib.positioning.position_hl_commander import PositionHlCommander
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
        self.first =False
        self.end = False
        self.first_pos=None
        self.last_pos=None
        self.first_time=0
        
    def rise(self, distance):
        self.pc.up(distance)

    def land(self):
        self.pc.land()
        return
    
    def forward(self, distance):
        self.pc.forward(distance)
        return
    
    def backward(self,distance):
        self.pc.back(distance)
        return
    
    def left(self, distance):
        self.pc.left(distance)
        return
    
    def right(self, distance):
        self.pc.right(distance)
        return
    
    def reset(self):
        self.first =False
        self.end = False
        self.first_pos=None
        self.last_pos=None
        self.first_time=0
        return
    
    def move(self, direction, distance):
        if direction =="front":
            self.forward(distance)
        elif direction == "right":
            self.right(distance)
        elif direction == "back":
            self.backward(distance)
        elif direction == "left":
            self.left(distance)
    
    def get_Position(self):
        return self.pc.get_position()

    #TODO 
    def is_Wall(self, path="front",distance=.7,back=True):
        directions = ["front","right","back","left"] #create clockwise rotation method
        location = directions.index(path) #find the which direction obstacle is in 
        position=self.get_Position()
        if range_dict["range."+directions[(location+1)%4]][-1]<1200*distance:
            if range_dict["range."+directions[(location-1)%4]][-1]<1200*distance:
                self.move(directions[(location-2)%4],distance)
                if range_dict["range."+directions[(location+1)%4]][-1]>1200*distance:
                    self.go_Around(directions[(location+1)%4],start=position)
                    value = range_dict["range."+path][-1]<550
                    if back or value:
                        self.go_Around(directions[(location-1)%4])
                elif range_dict["range."+directions[(location-1)%4]][-1]>1200*distance:
                    self.go_Around(directions[(location-1)%4],start=position)
                    value = range_dict["range."+path][-1]<550
                    if back or value:
                        self.go_Around(directions[(location+1)%4])
                else:
                    self.move(path,distance)
                    return True
            else:
                self.move(directions[(location-1)%4],distance)
                value = range_dict["range."+path][-1]<550
                if back or value:
                    self.move(directions[(location+1)%4],distance) 
        else:        
            self.move(directions[(location+1)%4],distance)
            value = range_dict["range."+path][-1]<550
            if back or value:
                self.move(directions[(location-1)%4],distance)
        #determine if obstacle in front is the wall or something to go around
        return value
    

    def go_Around(self, path="front", gap=500,start=None):
        directions = ["front","right","back","left"] #create clockwise rotation method
        location = directions.index(path) #find the which direction obstacle is in
        cord = location%2
        if start ==None:
            init_pos=self.get_Position()[cord]
        else:
            init_pos=start[cord]
        side = "Left"
        while range_dict["range."+path][-1]<gap*1.5: #while the obstacle is still in that direction
            if side=="Left" and range_dict["range."+directions[(location+1)%4]][-1]<gap: #if there is something in the clockwise direction
                position=self.get_Position()
                if self.is_Wall(directions[(location+1)%4],back=False):
                    side = "Right"
                    continue
                else:
                    self.go_Around(path=directions[(location+1)%4],start=position) #recall method (Cases still need to be fully tested)
                continue
            elif side=="Left":
                self.move(directions[(location+1)%4],.1) #move in the next directions
            elif side=="Right" and range_dict["range."+directions[(location-1)%4]][-1]<gap:
                position=self.get_Position()
                if self.is_Wall(directions[(location-1)%4],back=False):
                    side = "STUFF"
                    continue
                else:
                    self.go_Around(path=directions[(location-1)%4],start=position) #recall method (Cases still need to be fully tested)
                continue
            elif side=="Right":
                self.move(directions[(location-1)%4],.1)
            else:
                return False
        self.move(path,1.5*gap/1000) #move so that device is not in corner
        if "Left":
            flag =1
        elif "Right":
            flag = -1
        while range_dict["range."+directions[(location-(1*flag))%4]][-1]<gap: #while the obstacle is in the previous clockwise direction
            if range_dict["range."+path][-1]<gap:
                position =self.get_Position()
                if self.is_Wall(path,back=False):
                    return False
                self.go_Around(path,start=position)
                continue
            self.move(path,.1) #go the original direction
        moved =self.get_Position()[cord]-init_pos
        while abs(moved)>(gap-100)/1000:
            print(moved)
            print(((3*cord)+2+(abs(moved)/moved))%4)
            if range_dict["range."+directions[int(((3*cord)+2+(abs(moved)/moved))%4)]][-1]<gap: 
                print("WHY")
                if range_dict["range."+path][-1]<gap:
                    position = self.get_Position()
                    if self.is_Wall(path,back=False):
                        return False
                    self.go_Around(path,start=position)
                    moved =self.get_Position()[cord]-init_pos
                    continue
                self.move(path,.1)
                moved =self.get_Position()[cord]-init_pos
                continue
            self.move(directions[int(((3*cord)+2+(abs(moved)/moved))%4)],(gap-100)/1000)
            print("good")
            moved =self.get_Position()[cord]-init_pos
        self.move(directions[int(((3*cord)+2+(abs(moved)/moved))%4)],abs(moved))
        return True

    #TODO 
    def search_Land(self, distance=200, at_l_wall = False, at_r_wall = False):
        #assuming we start in the right corner of the section: -------------
                                                   #                      * |
                                                   #                        |       
        #while the landing area is not found, the drone will move left, then back, then right, then back .
        # This will repeat until landing pad is found
                 
        while not self.first:
            while not self.first and not at_l_wall:
                if range_dict["range.left"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around()
                        continue
                    else:
                        at_l_wall = True
                        break

                self.pc.left(.2)

            if not self.first:
                if range_dict['range.back'][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around('back')
                self.pc.back()
                        
            while not self.first and not at_r_wall:
                if range_dict["range.right"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around('right')
                        continue
                    else:
                        at_r_wall = True
                        break

                self.pc.right(.2)
            
            if not self.first:
                if range_dict['range.back'][-1] < 500:
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
    
    #JI
    def go_to(self,distance):
        #travels the distance needed avoiding obstacles
        
        start_pos = self.get_Position()
        target_pos = [start_pos[0]+distance[0], start_pos[1]+distance[1], start_pos[2]]
        current_pos = start_pos
        
        #go to y coordinate (vertical)
        while abs(target_pos[1]-current_pos[1])>=.1:
            self.move("front",.4) #move forward
            current_pos = self.get_Position() #get current position
            
            if range_dict["range.front"][-1]<=500: #if there's an obstacle
                self.go_around() #go around it

                current_pos = self.get_position()
        
        #go to x coordinate (left/right)
        while abs(target_pos[0]-current_pos[0])>=.1:
            if target_pos[0]-current_pos[0]>0:
                self.move("right",.4)
                current_pos = self.get_position()

                if range_dict["range.right"][-1]<=500:
                    self.go_around() #will go around still work if the object is on its right?
                    current_pos = self.get_position()

            elif target_pos[0]-current_pos[0]<0:
                self.move("left",.4)
                current_pos = self.get_position()

                if range_dict["range.left"][-1]<=500:
                    self.go_around() #will go around still work if the object is on its left?
                    current_pos = self.get_position()
        
        #go to y coordinate again (vertical)
        while abs(target_pos[1]-current_pos[1])>=.1:
            self.move("front",.4) #move forward
            current_pos = self.get_position() #get current position
            
            if range_dict["range.front"][-1]<=500: #if there's an obstacle
                self.go_around() #go around it
                current_pos = self.get_position()
        return

    #TODO 
    def search_Start(self, distance = .2, increase = .1, start_disance = None):
        #at assumed start position
        #if not above box
        #search for box using growing circles
        #break when find box

        #general motion: go forward(x) go_right(x) go_back(x+ delta) go_left(x+delta), x = x + 2*delta and repeat

        #if alredy above box, execute find center function (may need to be replaced with edge finding algo)
        if self.first:  
            return
        else:
            while not self.first:
                #forward
                if range_dict["range.front"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around()
                    else:
                        self.go_to(start_disance)
                        #figure out what to do here
                        #go back to start and decrease distance/increase parameter?
                        pass
                elif not self.first:
                    self.forward(distance)
                
                #right
                if range_dict["range.right"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around(path = 'right')
                    else:
                        self.go_to(start_disance)
                        #figure out what to do here
                        pass
                elif not self.first:
                    self.right(distance)

                #back
                if range_dict["range.back"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around(path='back')
                    else:
                        self.go_to(start_disance)
                        #figure out what to do here
                        pass
                elif not self.first:
                    self.backward(distance+increase) 
                
                #left
                if range_dict["range.left"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around(path='left')
                    else:
                        self.go_to(start_disance)
                        #figure out what to do here
                        pass
                elif not self.first:
                    self.left(distance+increase)

                distance = distance + (2*increase)
        
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
            if state_estimate_dict["stateEstimate.z"][-1]<.53 and not self.first:
                self.first =True
                self.first_pos=self.get_Position()
                self.first_time = time.time()
                print("first")
            if state_estimate_dict["stateEstimate.z"][-1]>.65 and self.first and self.first_time+1<time.time() and not self.end:
                self.end =True
                self.end_pos=self.get_Position()
                print("end")

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
    try:
        cflib.crtp.init_drivers()
        logs = init_logs()
        #for movement simplicity everything should be right angles
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            with PositionHlCommander(
                scf,
                #x=0.0, y=0.0, z=0.0,
                default_velocity=0.1,
                default_height=0.6,
                controller=PositionHlCommander.CONTROLLER_PID) as pc:
                drone = Drone(scf, pc)
                #initial_pos = drone.get_Position()
                drone.log_async(logs)
                print("takeoff")
                time.sleep(1)
                initial_pos = drone.get_Position()
                #time.sleep(5)
                for i in range(10):
                    drone.forward(.2)
                    if range_dict["range.left"][-1]<100:
                        drone.left(.1)
                    if range_dict["range.right"][-1]<100:
                        drone.right(.1)
                    if range_dict["range.front"][-1]<500:
                        print("STOP")
                        position = drone.get_Position()
                        if drone.is_Wall("front", back=False):
                            print("wall")
                        else:
                            print("no wall")
                            drone.go_Around(start=position)
                        #drone.backward(.2)
                        break
        #   TODO
        #   Fly code 
    #         at_end = False
    #         while not at_end:
    #             if range_dict["range.front"][-1]<500:
    #                 position = drone.get_Position()
    #                 if drone.get_Position()[0]>1 and drone.is_wall(back=False):
    #                     at_end=True
    #                     break
    #                 if not drone.go_Around(start=position):
    #                     at_end=True
    #                     break
    #                 continue
    #             drone.forward(.2)
    #         drone.search_Land()
    #         drone.find_center()
    #         end_pos= drone.get_Position()
    #time.sleep(2)
    #with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    #        with PositionHlCommander(
    #            scf,
    #            x=end_pos[0], y=end_pos[1], z=end_pos[2],
    #            default_velocity=0.1,
    #            default_height=0.6,
    #            controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
    #         drone = Drone(scf,mc)
    #         distance=[initial_pos[0]-end_pos[0],initial_pos[1]-end_pos[1]]
    #         drone.go_to(distance)
    #         drone.search_Start(start_distance = distance)
    #         drone.go_to(distance)
    #         drone.search_Start()
    #         drone.find_center()
    except InterruptedError:
        print("ERROR") 
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

