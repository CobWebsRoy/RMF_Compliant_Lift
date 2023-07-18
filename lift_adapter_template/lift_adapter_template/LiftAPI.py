#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import sys
import enum
from yaml import YAMLObject
from typing import Optional
from rmf_lift_msgs.msg import LiftState
from rclpy.impl.rcutils_logger import RcutilsLogger

#FOR SERVOS ON I2C PCA9685
from adafruit_servokit import ServoKit
from time import sleep
kit=ServoKit(channels=16)
servo=16
#-------------------------

class DoorState(enum.IntEnum):
    CLOSED = 0
    MOVING = 1
    OPEN = 2


class MotionState(enum.IntEnum):
    STOPPED = 0
    UP = 1
    DOWN = 2
    UNKNOWN = 3


'''
    The LiftAPI class is a wrapper for API calls to the lift. Here users are
    expected to fill up the implementations of functions which will be used by
    the LiftAdapter. For example, if your lift has a REST API, you will need to
    make http request calls to the appropriate endpints within these functions.
'''
class LiftAPI:
    # The constructor accepts a safe loaded YAMLObject, which should contain all
    # information that is required to run any of these API calls.
    def __init__(self, config: YAMLObject, logger: RcutilsLogger):
        self.config = config
        self.logger = logger


	#INITALIZING ALL SERVOS TO NEUTRAL POSITION        
        kit.servo[0].angle=90
        kit.servo[1].angle=90
        kit.servo[2].angle=90
	#------------------------------------------
	
	
        # Test initial connectivity
        self.logger.info('Checking connectivity.')
        if not self.check_connection():
            self.logger.error('Failed to establish connection with lift API')
            sys.exit(1)

    def check_connection(self) -> bool:
        ''' Return True if connection to the lift is successful'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True

    def available_floors(self) -> Optional[list[str]]:
        ''' Returns the available floors for this lift, or None the query
            failed'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        
        available_floors = ['0', '1', '2', '3', '4']
        return available_floors
        
        # ------------------------ #
        #return None

    def current_floor(self) -> Optional[str]:
        ''' Returns the current floor of this lift, or None the query failed'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        
        current_floor = None
        return current_floor
        
        # ------------------------ #
        #return None

    def destination_floor(self) -> Optional[str]:
        ''' Returns the destination floor of this lift, or None the query
            failed'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        
        destination_floor = None
        return destination_floor
        
        # ------------------------ #
        #return None

    def lift_door_state(self) -> Optional[int]:
        ''' Returns the state of the lift door, based on the static enum
            LiftState.DOOR_*, or None the query failed'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        
        #door_state = None
        #return door_state.value
        
        # ------------------------ #
        #return None

    def lift_motion_state(self) -> Optional[int]:
        ''' Returns the lift cabin motion state, based on the static enum
            LiftState.MOTION_*, or None the query failed'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        
        motion_state = MotionState.UNKNOWN
        return motion_state.value
        
        # ------------------------ #
        return None

    def command_lift(self, floor: str) -> bool:
        ''' Sends the lift cabin to a specific floor and opens all available
            doors for that floor. Returns True if the request was sent out
            successfully, False otherwise'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        
        if int(floor) == 0:
            kit.servo[0].angle = 90
            return True

        elif int(floor) == 1:
            kit.servo[0].angle = 90
            kit.servo[1].angle = 150
            sleep(2)
            kit.servo[1].angle = 90
            return True
        
        elif int(floor) == 2:
            kit.servo[0].angle = 90
            kit.servo[2].angle = 150
            sleep(2)
            kit.servo[2].angle = 90
            return True
          
        elif int(floor) == 3:
            kit.servo[0].angle = 90
            kit.servo[3].angle = 150
            sleep(2)
            kit.servo[3].angle = 90
            return True
        
        elif int(floor) == 4:
            kit.servo[0].angle = 90
            kit.servo[4].angle = 150
            sleep(2)
            kit.servo[4].angle = 90
            return True
            
        
        # ------------------------ #
        return False
        
    def command_lift_door(self, door: str):
        if int(door) == 1:
            kit.servo[0].angle = 150
        elif int(door) == 0:
            kit.servo[1].angle = 90
            
