from enum import Enum
import time

import rclpy
from .robot_controller import RobotController
from .robot_controller import NavigationResult

class State(Enum):
    INIT = 0
    GO_WAIT_IN = 1
    REQUEST_IN = 2
    WAIT = 3
    GO_UNLOAD = 4
    UNLOADING = 5
    GO_WAIT_OUT = 6
    NOTIFY_OUT = 7
    GO_LOAD = 8
    LOADING = 9
    CHECK_GOAL = 10
    IDLE = 11

def main():
    rclpy.init()

    navigator = RobotController()    

    state = State.INIT
    nextState = None
    prevState = None

    while True:
        navigator.debug(f'Current state = {state}')
        if state == State.INIT:
            # Wait for navigation to fully activate, since autostarting nav2
            navigator.waitUntilNav2Active()
            # navigator.waitTrafficControlCenterActive()
            state = State.LOADING

        elif state == State.GO_WAIT_IN:
            navigator.info('Go WAITIN position')

            navigator.goWaitInPose()
            prevState = state
            nextState = State.REQUEST_IN
            state = State.CHECK_GOAL

        elif state == State.REQUEST_IN:
            navigator.info('Send request to traffic control server')
            navigator.requestInPermission()
            state = State.WAIT

        elif state == State.WAIT:
            navigator.info('Wait traffic control server send permission')            
            navigator.waitPermision()
            navigator.info('Received traffic control server send permission')
            state = State.GO_UNLOAD
        
        elif state == State.GO_UNLOAD:
            navigator.info('Go UNLOAD position')
            navigator.goUnloadPose()
            prevState = state
            nextState = State.UNLOADING
            state = State.CHECK_GOAL

        elif state == state.UNLOADING:
            navigator.info('Start unloading packages...')
            time.sleep(3)
            navigator.info('Packages unloaded done, return to load position')
            state = State.GO_WAIT_OUT

        elif state == State.GO_WAIT_OUT:
            navigator.info('Go WAIT_OUT position')
            navigator.goWaitOutPose()
            prevState = state
            nextState = State.NOTIFY_OUT
            state = State.CHECK_GOAL
        
        elif state == State.NOTIFY_OUT:
            navigator.info('Notify traffic control complete unloading')
            navigator.notifyTrafficControlComplete()
            state = State.GO_LOAD
        
        elif state == State.GO_LOAD:
            navigator.info('Go LOAD position')
            navigator.goInitialPose()
            prevState = state
            nextState = State.LOADING
            state = State.CHECK_GOAL
        
        elif state == State.LOADING:
            navigator.info('Start loading packages...')
            time.sleep(3)
            navigator.info('Packages loading done, go to unload position')
            state = State.GO_WAIT_IN
        
        elif state == State.CHECK_GOAL:
            if not navigator.isNavComplete():
                time.sleep(2)
            else:
                # Do something depending on the return code
                result = navigator.getResult()
                if result == NavigationResult.SUCCEEDED:
                    navigator.info('Goal succeeded!')
                    state = nextState
                elif result == NavigationResult.CANCELED:
                    navigator.info('Goal was canceled!')
                    state = State.IDLE
                elif result == NavigationResult.FAILED:
                    navigator.error('Goal failed!')
                    navigator.cancelNav()
                    navigator.info('Retry send goal!')
                    state = prevState
                else:
                    navigator.info('Goal has an invalid return status!')
                    state = State.IDLE               
                
        else:
            navigator.error(f'Unknown state: {state}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()