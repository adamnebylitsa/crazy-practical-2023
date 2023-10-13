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
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E710')

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
        self.start =False
        self.end = False
        self.start_pos=None
        self.end_pos=None
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
    
    def reset(self): #resets the flags and parameters for finding the box
        self.start =False
        self.end = False
        self.start_pos=None
        self.end_pos=None
        self.first_time=0
        return
    
    def move(self, direction, distance): #general move function to allow for any path taken
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

    #method to determine if the path is a wall by moving a distance to the side and determining if the object ist still in front
    #method would default to going back to the original location and anytime it is a wall
    def is_Wall(self, path="front",distance=.7,back=True):
        # path is direction going, distance is max distance to move to the side to determine if it is a wall, 
        # back is a flag for if it would go back to the original location
        directions = ["front","right","back","left"] #create clockwise rotation method
        location = directions.index(path) #find the which direction obstacle is in 
        position=self.get_Position() # gets current position
        if range_dict["range."+directions[(location+1)%4]][-1]<1200*distance: #if there is a obstacle in the clockwise direction
            if range_dict["range."+directions[(location-1)%4]][-1]<1200*distance: #if there is also an obstacle in the counterclockwise direction
                self.move(directions[(location-2)%4],distance) #move backwards
                if range_dict["range."+directions[(location+1)%4]][-1]>1200*distance: #if there is nothing in the clockwise
                    self.go_Around(directions[(location+1)%4],start=position) #go around the object to the side
                    value = range_dict["range."+path][-1]<550 #test if something is still in front
                    if back or value:
                        self.go_Around(directions[(location-1)%4]) #go back if forced or if it is a wall
                elif range_dict["range."+directions[(location-1)%4]][-1]>1200*distance: #if something in the clockwise, check counter clockwise
                    self.go_Around(directions[(location-1)%4],start=position,way ="left") #go around the object
                    value = range_dict["range."+path][-1]<550 #test if something is still in front 
                    if back or value:
                        self.go_Around(directions[(location+1)%4]) #go back if forced or if it is a wall
                else:
                    self.move(path,distance) #if there is still an object on both sides return to original location and return its a wall
                    return True
            else: #if only object in clockwise location
                for i in range(10):
                    self.move(directions[(location-1)%4],distance/10)#move counter clockwise a max of distance
                    value = range_dict["range."+path][-1]<550 #see if something is in front
                    if not value: #if there is ever nothing in front
                        self.move(directions[(location-1)%4],.1) #move a little more and stop
                        break
                if back or value: #return if forced or still a wall
                    self.move(directions[(location+1)%4],distance) 
        else: #if nothing in the clockwise location
            for i in range(10):        
                self.move(directions[(location+1)%4],distance/10) #move in the clockwise direction a max of distance
                value = range_dict["range."+path][-1]<550 #see if something is in front
                if not value: #if there is ever nothing in front
                    self.move(directions[(location+1)%4],.1) #move a little more and stop
                    break
            if back or value: #return if forced or still a wall
                self.move(directions[(location-1)%4],distance)
        return value
    
    #method to go around an object, object must not be a wall, can work for every direction 
    #method would return to the original axis of motion
    def go_Around(self, path="front", gap=500,start=None,way="right"):
        #path is original way of motion, gap is the distance wanted between the object and the drone,
        #start is the start location, if none the position at the start of the method is used, way is the first direction it should try
        directions = ["front","right","back","left"] #create clockwise rotation method
        location = directions.index(path) #find the which direction obstacle is in
        cord = (location+1)%2 #find the cord (x,y) for the axis that should be unchanged
        if start ==None: #get initial position for the start of the method or if given
            init_pos=self.get_Position()[cord]
        else:
            init_pos=start[cord]
        while range_dict["range."+path][-1]<gap*1.2: #while the obstacle is still in that direction
            if way=="right" and range_dict["range."+directions[(location+1)%4]][-1]<gap: #if there is something in the clockwise direction
                position=self.get_Position()
                if self.is_Wall(directions[(location+1)%4],back=False): #and that object is a wall
                    way="left" #switch the direction
                    continue
                else:
                    self.go_Around(path=directions[(location+1)%4],start=position) #if it is not a wall go around that as well
                continue
            elif way =="right": # if you could go clockwise
                self.move(directions[(location+1)%4],.1) #move in that directions
            elif way=="left" and range_dict["range."+directions[(location-1)%4]][-1]<gap: #if trying to go counter clockwise and there is something
                position=self.get_Position()
                if self.is_Wall(directions[(location-1)%4],back=False): #and that object is a wall
                    way = "STUFF" #then either direction is a wall and you can not go around
                    continue
                else:
                    self.go_Around(path=directions[(location-1)%4],start=position) #recall method
                continue
            elif way=="left": #if trying to go counterclockwise go that direction
                self.move(directions[(location-1)%4],.1)
            else: #if can not go clockwise or counter clockwise return false as a flag that it failed to go around
                return False
        self.move(path,1.2*gap/1000) #move so that device is not in corner and can sense the object it tried to go around
        if "Left": #flag for which direction the object would no be on.
            flag =1
        elif "Right":
            flag = -1
        while range_dict["range."+directions[(location-(1*flag))%4]][-1]<gap: #while the obstacle is on the side of the drone
            if range_dict["range."+path][-1]<gap: #if there is something in front
                position =self.get_Position()
                if self.is_Wall(path,back=False): #if it is a wall return that the method failed
                    return False
                self.go_Around(path,start=position) #otherwise go around that as well
                continue
            self.move(path,.1) #when nothing in front go the original direction
        moved =self.get_Position()[cord]-init_pos #find the distance that the no affected cord went
        while abs(moved)>(gap-100)/1000: #while the distance is larger than the gap
            print(moved)
            print(((3*cord)+1+(abs(moved)/moved))%4) #math to get the proper direction to go based on cord and distance moved
            if range_dict["range."+directions[int(((3*cord)+1+(abs(moved)/moved))%4)]][-1]<gap: #if there is something in that direction
                if range_dict["range."+path][-1]<gap: #test if something is in the original direction to keep going there
                    position = self.get_Position()
                    if self.is_Wall(path,back=False): #test if wall, and fail method if needed
                        return False
                    self.go_Around(path,start=position) #otherwise it can go around
                    moved =self.get_Position()[cord]-init_pos
                    continue
                self.move(path,.1) #when nothing in original path, go that direction a little more
                moved =self.get_Position()[cord]-init_pos
                continue
            self.move(directions[int(((3*cord)+1+(abs(moved)/moved))%4)],(gap-100)/1000) #when nothing in desired direction go that direction
            print("good")
            moved =self.get_Position()[cord]-init_pos
        self.move(directions[int(((3*cord)+1+(abs(moved)/moved))%4)],abs(moved)) #move remaining amount
        return True #return that the method was successful


    def search_Land(self, distance=200, at_l_wall = False, at_r_wall = False):
        #assuming we start in the right corner of the section: -------------
                                                   #                      * |
                                                   #                        |       
        #while the landing area is not found, the drone will move left, then back, then right, then back .
        # This will repeat until landing pad is found
                 
        while not self.start: #will did not go over the box
            while not self.start and not at_l_wall: #if it did not reach the left
                if range_dict["range.left"][-1] < 500: #If there is something on the left
                    if not self.is_Wall(): #see if it is a wall
                        self.go_Around("left") #go around if needed
                        continue
                    else:
                        at_l_wall = True #mark that you reached the left wall
                        break

                self.pc.left(.2)

            if not self.start: 
                if range_dict['range.back'][-1] < 500:#if nothing is behind go back
                    if not self.is_Wall():
                        self.go_Around('back')
                self.pc.back(.2)
                        
            while not self.start and not at_r_wall: #if it did not reach the right wall
                if range_dict["range.right"][-1] < 500: #check if something is on the right
                    if not self.is_Wall():
                        self.go_Around('right') #go around if needed
                        continue
                    else: #if it is a wall mark that the wall it hit
                        at_r_wall = True
                        break

                self.pc.right(.2) #if there is nothing to the right go right
            
            if not self.start: #if it did not reach the box go back to continue the search
                if range_dict['range.back'][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around('back')
                self.pc.back(.2)

        #repeated until it reaches the box
        
        return
       #method to find and go to the center of the box assuming it is already over the box
    def find_center(self):
        self.backward(.3)#go back a little so it is not over the box
        self.reset() #reset the flags
        print("forward")
        while not self.end: #go forward until it reaches the end of the box
            self.forward(.2)
        time.sleep(.5)
        midpoint = [(self.end_pos[0]+self.start_pos[0])/2,(self.end_pos[1]+self.start_pos[1])/2] #calculate the midpoint
        drone.pc.go_to(midpoint[0],midpoint[1]) #go to the midpoint
        #current = self.get_Position()
        #self.go_to([midpoint[0]-current[0],midpoint[1]-current[1]])
        print("mid 1")
        time.sleep(.5)
        #go to the left so it is no longer over the box and finds the middle in the other axis
        self.left(.3)
        self.reset()# reset the flags
        print("right")
        while not self.end: #while it did not reach the end go right
            self.right(.2)
        time.sleep(.5)
        midpoint = [(self.end_pos[0]+self.start_pos[0])/2,(self.end_pos[1]+self.start_pos[1])/2] #calculate the midpoint
        #current = self.get_Position()
        #self.go_to([midpoint[0]-current[0],midpoint[1]-current[1]])
        drone.pc.go_to(midpoint[0],midpoint[1]) #go to the midpoint
        print("mid 2")
        return
    
    #method to go to a location when distance is a list of distance it needs to travel
    def go_to(self,distance):
        #travels the distance needed avoiding obstacles
        start_pos = self.get_Position() #get to the current 
        target_pos = [start_pos[0]+distance[0], start_pos[1]+distance[1], start_pos[2]] # get the desired end point
        current_pos = start_pos
        
        #go to x coordinate (front/back)
        while abs(target_pos[0]-current_pos[0])>=.4: #if there is a distance in the x direction
            if target_pos[0]-current_pos[0]>0: #if it is forward
                if range_dict["range.front"][-1]<=500: #if there's an obstacle
                    self.go_Around() #go around it
                    current_pos = self.get_Position()
                    continue
                self.move("front",.4) #move forward
            elif target_pos[0]-current_pos[0]<0: #if it is backwards
                if range_dict["range.back"][-1]<=500: #if there's an obstacle
                    self.go_Around(path="back") #go around it
                    current_pos = self.get_Position()
                    continue
                self.move("back",.4)
            current_pos = self.get_Position() #get current position
        #do the remaining distance to travel
        if target_pos[0]-current_pos[0]>0:
            self.move("front",target_pos[0]-current_pos[0])
        elif target_pos[0]-current_pos[0]<0:
            self.move("back",target_pos[0]-current_pos[0])

        #go to y coordinate (left/right)
        while abs(target_pos[1]-current_pos[1])>=.4: #if there is a distance in the y direction
            if target_pos[1]-current_pos[1]<0: #if it goes right
                if range_dict["range.right"][-1]<=500:#if there is an obstacle
                    self.go_Around(path="right") #go around it
                    current_pos = self.get_Position()
                    continue
                self.move("right",.4)

            elif target_pos[1]-current_pos[1]>0: #if it goes left
                if range_dict["range.left"][-1]<=500: #if there is an obstacle
                    self.go_Around(path="left") #go around it
                    current_pos = self.get_Position()
                    continue
                self.move("left",.4)

            current_pos = self.get_Position()#get current position
        #move the remaining distance to travel
        if target_pos[1]-current_pos[1]>0:
            self.move("left",target_pos[1]-current_pos[1])
        elif target_pos[1]-current_pos[1]<0:
            self.move("right",target_pos[1]-current_pos[1])
        
        #go to x coordinate again (front/back)
        while abs(target_pos[0]-current_pos[0])>=.4:#if there is a distance to go in the x direction
            if target_pos[0]-current_pos[0]>0: #if the direction is forward
                if range_dict["range.front"][-1]<=500: #if there's an obstacle
                    self.go_Around() #go around it
                    current_pos = self.get_Position()
                    continue
                self.move("front",.4) #move forward
            elif target_pos[0]-current_pos[0]<0:#if the direction is backwards
                if range_dict["range.back"][-1]<=500: #if there's an obstacle
                    self.go_Around(path="back") #go around it
                    current_pos = self.get_Position()
                    continue
                self.move("back",.4)
            current_pos = self.get_Position() #get current position
        #do remaining movements
        if target_pos[0]-current_pos[0]>0:
            self.move("front",target_pos[0]-current_pos[0])
        elif target_pos[0]-current_pos[0]<0:
            self.move("back",target_pos[0]-current_pos[0])
        return

    #assuming the drone is at the estimated start position, start going around gradually increasing the circle,
    #this is used as the location would be close to the actual location
    def search_Start(self, distance = .2, increase = .1, start_disance = None):

        #general motion: go forward(x) go_right(x) go_back(x+ delta) go_left(x+delta), x = x + 2*delta and repeat

        #if alredy above box, execute find center function (may need to be replaced with edge finding algo)
        if self.start:  #if not already over the box
            return
        else:
            while not self.start:
                #forward
                if range_dict["range.front"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around()
                    else:
                        self.go_to(start_disance)
                        #figure out what to do here
                        #go back to start and decrease distance/increase parameter?
                        pass
                elif not self.start:
                    self.forward(distance)
                
                #right
                if range_dict["range.right"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around(path = 'right')
                    else:
                        self.go_to(start_disance)
                        #figure out what to do here
                        pass
                elif not self.start:
                    self.right(distance)

                #back
                if range_dict["range.back"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around(path='back')
                    else:
                        self.go_to(start_disance)
                        #figure out what to do here
                        pass
                elif not self.start:
                    self.backward(distance+increase) 
                
                #left
                if range_dict["range.left"][-1] < 500:
                    if not self.is_Wall():
                        self.go_Around(path='left')
                    else:
                        self.go_to(start_disance)
                        #figure out what to do here
                        pass
                elif not self.start:
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
            if state_estimate_dict["stateEstimate.z"][-1]<.53 and not self.start:
                self.start =True
                self.start_pos=self.get_Position()
                self.start_time = time.time()
                print("start")
            if state_estimate_dict["stateEstimate.z"][-1]>.65 and self.start and self.first_time+1<time.time() and not self.end:
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
    cflib.crtp.init_drivers()
    logs = init_logs()
    #for movement simplicity everything should be right angles
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        pc=PositionHlCommander(
            scf,
            #x=0.0, y=0.0, z=0.0,
            default_velocity=0.1,
            default_height=0.6,
            controller=PositionHlCommander.CONTROLLER_MELLINGER)
        drone = Drone(scf, pc)
        try: 
            drone.log_async(logs) #start logging data
            drone.pc.take_off() #take off 
            print("takeoff")
            time.sleep(1)
            initial_pos = drone.get_Position() #get initial position
            print(initial_pos)
            #time.sleep(5)
        #   Fly code 
            at_end = False 
            while not at_end: #while it did not reach the end of the room
                if range_dict["range.front"][-1]<500: #if there is something in front
                    print("Stop")
                    position = drone.get_Position()
                    if drone.get_Position()[0]>1 and drone.is_Wall(back=False): #if it over 1m away it could be a wall
                        print("wall") #if it is a wall mark that the end is reached
                        at_end=True
                        break
                    if not drone.go_Around(start=position): #if it is not a wall go around, if it ever fails mark it as reached the end
                        print("around")
                        at_end=True
                        break
                    continue
                if range_dict["range.left"][-1]<100: #if there is something to the left move right a little
                    print("right")
                    drone.right(.1)
                if range_dict["range.right"][-1]<100: #if there is something to the right move left a little
                    print("left")
                    drone.left(.1)
                drone.forward(.2) #if there is nothing in front move forward
            while range_dict["range.right"][-1]<400: #once the end is reached if there is nothing to the right go right
                drone.right(.2)
            print("corner")#if there is something to the right the top right corner is reached
            drone.reset() #reset the flags for finding the box
            print("searching")
            drone.search_Land() #search for the box
            #drone.find_center() #find the center of the box
            print("land")
            time.sleep(.5)
            drone.pc.down(.6) #land on the box
            drone.pc.land()
            end_pos= drone.get_Position() #get the end position
            print("wait")        
            time.sleep(2)
            drone.pc.take_off()#take off from the box
            print("take off")
            distance=[initial_pos[0]-end_pos[0],initial_pos[1]-end_pos[1]] #find the distance the drone needs to travel back
            drone.go_to(distance) #travel back to start
            print("at Start")
            drone.search_Start(start_distance = distance) #search for the starting location
            time.sleep(.5)
            #drone.find_center() #find the center of the box
            print("landing")
            drone.pc.down(.6) #land on the box
            drone.pc.land()
        except KeyboardInterrupt: #Safety feature that if something goes wrong a keyboard interruption can be used and the drone would land
            #without this the drone would continue to hover, even with the radio disconnected
            drone.pc.land()
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

