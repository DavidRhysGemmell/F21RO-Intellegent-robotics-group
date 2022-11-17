from controller import Robot
from datetime import datetime
import math
import numpy as np
import sys
class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0
    
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name)) #higher number, closer object
            self.proximity_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
        # Data
        self.inputs = []
        self.inputsPrevious = []
        
        #black
        self.black=False #set at start no black detected
        
        
    def black_square(self):
          if self.center_ir.getValue()<500: #If black (gs<500) is detected, black=True
              self.black=True
              print('black detected on floor')
                                     
    def run_robot(self):        
        # Main Loop
        count = 0;
        inputs_avg = []
        while self.robot.step(self.time_step) != -1:
            # Read Ground Sensors
            self.inputs = []
            left = self.left_ir.getValue() #black is below 500, white above 500
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()
            
            min_gs = 0 #minimum ground sensor output
            max_gs = 1000 # maximum ground sensor output
            
            #this just makes sure all the values are within min_gs and max_gs
            if(left > max_gs): left = max_gs 
            if(center > max_gs): center = max_gs
            if(right > max_gs): right = max_gs
            if(left < min_gs): left = min_gs
            if(center < min_gs): center = min_gs
            if(right < min_gs): right = min_gs

            
            # Read Distance Sensors

            distance_left = self.proximity_sensors[5].getValue()
            distance_right= self.proximity_sensors[2].getValue()
            distance_center= (self.proximity_sensors[0].getValue()+self.proximity_sensors[7].getValue())/2       
            
            
            #Checks if a black has been detected
            if self.black==True:                   
                distance_center= self.proximity_sensors[7].getValue() #if black has been detected we take the front left sensor as the center sensor. This stops the bot oversteering once it's turned right.
                
            else:
                distance_center= self.proximity_sensors[0].getValue()#if black has not been detected we take the front right sensor as the center sensor. This stops the bot oversteering once it's turned left.

            if abs(distance_left - distance_right)<100: #If the difference between the left and right sensors is relatively equal the bot will drive straight.
                self.velocity_left = 1
                self.velocity_right = 1
            elif distance_left>distance_right: #If the left wall is closer than the right wall (within margin) then the bot will turn right. Note, greater number = closer.
                self.velocity_left = 1
                self.velocity_right = 0.5
            elif  distance_left<distance_right:#If the right wall is closer than the left wall (within margin) then the bot will turn left. Note, greater number = closer.
                self.velocity_left = 0.5
                self.velocity_right = 1
            
            if distance_center>100:   #If the frontal proximity sensors detect an object close to it              
                if self.black==True: #If it's seen black the bot will turn right
                    self.velocity_left = 1
                    self.velocity_right = -1
                else: #If it has not seen black then the bot will turn left
                    self.velocity_left = -1
                    self.velocity_right = 1
            self.black_square()
              #Termination criteria. If the robot reaches the end of the corridor (all proximity sensors, except the back two, detect the walls as sufficiently close)  it will stop.          
            if distance_left >100 and distance_right > 100 and self.proximity_sensors[6].getValue() > 100 and self.proximity_sensors[7].getValue()>100 and self.proximity_sensors[0].getValue()>100 and self.proximity_sensors[1].getValue()>100:
                self.velocity_left = 0
                self.velocity_right = 0
                self.left_motor.setVelocity(self.velocity_left) # publishes velocities to the motors   
                self.right_motor.setVelocity(self.velocity_right)  # publishes velocities to the motors    
                print('reached goal')
                sys.exit("System Exiting")
                print("Program End")                 
                              
            self.left_motor.setVelocity(self.velocity_left) # publishes velocities to the motors
            self.right_motor.setVelocity(self.velocity_right)  # publishes velocities to the motors     
              
                
            
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
