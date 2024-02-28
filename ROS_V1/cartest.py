'''
RL PROJECT
By Richard Sowers
* <r-sowers@illinois.edu>
* <https://publish.illinois.edu/r-sowers/>

Copyright 2023
University of Illinois Board of Trustees. All Rights Reserved.

Code implements simplified version of https://github.com/openai/gym/blob/master/gym/envs/classic_control/continuous_mountain_car.py
'''

#imports
import numpy
import math
import sys
import py_compile

#main
class Valley:
    @staticmethod
    def height(x):
        return 0.45*numpy.sin(3.1*x)+0.55+0.25*numpy.exp(-50*(x+0.5)**2)
    
    @staticmethod
    def slope(x):
        return 0.45*3.1*numpy.cos(3.1*x)+0.25*50*(-2)*(x+0.5)*numpy.exp(-50*(x+0.5)**2)

class Car():
    def __init__(self,initial_position=0,initial_velocity=0):
        self.x_min=-1.2
        self.x_max=0.6
        self.min_action=-1
        self.max_action=1
        self.mass=1.1
        self.dt=0.05
        self.friction=0.01
        initial_position=float(initial_position)
        initial_velocity=float(initial_velocity)
        self.reset(initial_position=float(initial_position),initial_velocity=float(initial_velocity))
        
    def terminated_check(self):
        (position,velocity)=self.state
        self.terminated=(position<=self.x_min) or (position>=self.x_max)
    

    def step(self, action):
        action=max(min(action,self.max_action),self.min_action) #threshold the action

        (position,velocity)=(self.state[0],self.state[1])
        if self.terminated: #don't update if terminated
            self.state=(position,0)
            return self.state,None,self.terminated
        
        force=action+Valley.height(position)-self.friction*numpy.sign(velocity) #action+gravitational force+friction

        velocity += force +self.dt/self.mass*force #m(dv/dt)=force
        position+=velocity*self.dt #(dx/dt=v)
        self.state=(position,velocity)
        
        self.terminated_check()
    
        reward=-0.1*self.dt-0.1*action**2 #penalty for continuing is time plus energy used
        return self.state, reward, self.terminated

    def reset(self,initial_position=0,initial_velocity=0):
        self.state=(initial_position,initial_velocity)
        print("initializing to initial position={0:f} and initial_velocity={1:f}".format(initial_position,initial_velocity))
        self.terminated_check()
