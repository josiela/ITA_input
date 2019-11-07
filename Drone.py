import math
import time

import logging

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger





#small version of the drone to only test with the LEDS to avoid all the estimator code
#Cannot be used to fly!
class BaseDrone ():
    """ BaseDrone class that is used to communicate to a crazyflie2.1 drone.

    The BaseDrone class establishes a connection and gives a small set of functions to verify that the connection is set e.g. communicating with the led ring.
    """


################################################################################
#               Initialization and cleanup code
################################################################################

    def __init__(self, uri):
        """Initialize the connection to the Drone.

        The uri is depending on the drone and has to be given at initialization. 
        """
        self.uri = uri

        cflib.crtp.init_drivers(enable_debug_driver=False)

        self.cf = Crazyflie(rw_cache='./cache')
        self.scf = SyncCrazyflie (self.uri, cf=self.cf)

    def __enter__(self):
        """Establishes the link to the drone and returns the BaseDroneObject"""
        self.scf.open_link()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Closes the connection to the drone to secure a clean shutdown even in case of an exeption"""
        self.close_link()

    def close_link(self):
        """Closes the Link to the drone and removes possible callbacks"""
        self.scf.cf.close_link()
        self.scf._remove_callbacks()
        self.scf._is_link_open = False


################################################################################
#        Higher Functionality like setting the color of the LED's
#                       (get creative here)
################################################################################

    def red_green_color(self, sequence):
        """Gets an array of values and displays a green or a red light on the led ring

        if sequence[n] - startXPositionOfDrone < 0: red light
        else: green light
        """
        cf = self.scf.cf
        
        self.setRingEffect(7)
        
        for position in sequence:
            x = position[0]
            if x<0:
                self.setRingColor(red=100)
            else:
                self.setRingColor(green=100)
            time.sleep(1) # time for led to be shown

    def setRingEffect(self, value):
        """Sets the ring.effect param of the cf to the specified value
        
        If the value is not in range of the specified effects (see https://wiki.bitcraze.io/projects:crazyflie2:expansionboards:ledring), it will print an warning
        """
        
        MAX_NUM_EFFECTS = 13
        if value >= 0 and value < MAX_NUM_EFFECTS:
            self.scf.cf.param.set_value('ring.effect',str(value))
        else:
            print("WARNING: Could not set ring.effect to value: {}. Use a value between 0 and {}. For further Information see https://wiki.bitcraze.io/projects:crazyflie2:expansionboards:ledring".format(value,MAX_NUM_EFFECTS))

    
    def setRingColor(self, red = 0, green = 0, blue = 0):
        """Sets the color of the ring to a specific value.

        Red, Green and Blue can be numbers between 0 and 100.
        TODO: test if ring effect has to be set to 7 all the time.
        """
        if red < 0 or red > 100 or green < 0 or green > 100 or blue < 0 or blue > 100:
            return False
        else:
            self.scf.cf.param.set_value('ring.solidRed', str(red))
            self.scf.cf.param.set_value('ring.solidGreen', str(green))
            self.scf.cf.param.set_value('ring.solidBlue', str(blue))
            return True




class Drone(BaseDrone):
    """Wrapper class for the crazyflie python lib.

    This class handles the init and cleanup of the connection to the drone.
    It also has some methods for some functionality like flying sequences.
    """

################################################################################
#               Initialization and cleanup code
################################################################################

    def __init__(self, uri, initial_x = 0, initial_y = 0, initial_z = 0, initial_yaw = 0):
        """Establishes the connection to the drone.

        Uses the uri parameter to connect to the corresponding drone. the initial_ values are used to save the starting position of the drone.
        """
        self.uri = uri
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_z = initial_z
        self.initial_yaw = initial_z  

        # Only output errors from the logging framework
        logging.basicConfig(level=logging.ERROR)
        
        cflib.crtp.init_drivers(enable_debug_driver=False)

        self.cf = Crazyflie(rw_cache='./cache')
        self.scf = SyncCrazyflie (self.uri, cf=self.cf)

    def __enter__(self):
        """opens the link to the drone and sets the initial position and resets the estimator. Returns the object of the Drone class
        """
        self.scf.open_link()
        self.set_initial_position()
        self.reset_estimator()
        return self


################################################################################
#                       Check for "safe" drone position
################################################################################

    def wait_for_position_estimator(self):
        """waits until the estimator has determined the position of the drones
        """
        print('Waiting for estimator to find position...')

        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001

        with SyncLogger(self.scf, log_config) as logger:
            for log_entry in logger:
                data = log_entry[1]

                var_x_history.append(data['kalman.varPX'])
                var_x_history.pop(0)
                var_y_history.append(data['kalman.varPY'])
                var_y_history.pop(0)
                var_z_history.append(data['kalman.varPZ'])
                var_z_history.pop(0)

                min_x = min(var_x_history)
                max_x = max(var_x_history)
                min_y = min(var_y_history)
                max_y = max(var_y_history)
                min_z = min(var_z_history)
                max_z = max(var_z_history)

                # print("{} {} {}".
                #       format(max_x - min_x, max_y - min_y, max_z - min_z))

                if (max_x - min_x) < threshold and (
                        max_y - min_y) < threshold and (
                        max_z - min_z) < threshold:
                    break


    def set_initial_position(self):
        """Sets the given intial variables to the drone params kalman.initial...
        """
        self.scf.cf.param.set_value('kalman.initialX', self.initial_x)
        self.scf.cf.param.set_value('kalman.initialY', self.initial_y)
        self.scf.cf.param.set_value('kalman.initialZ', self.initial_z)

        yaw_radians = math.radians(self.initial_yaw)
        self.scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


    def reset_estimator(self):
        """Resets the estimator of the drone. Calls wait_for_position_estimator afterwards"""
        cf = self.scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')

        self.wait_for_position_estimator()

################################################################################
#              Higher Functionality like flying a Sequence 
#                       (get creative here)
################################################################################

    #implement flyingSequence here
    def flySequence(self, sequence,time_between_commandos = 2, checkZSafety=True):
        """Lets the drone fly to the points specified in the sequence array.

        The sequence array should be like this: [(x1,y1,z1),(x2,y2,z2),...]
        The time_between_commandos parameter specifies the time the drone has to execute the fly command. (TODO: maybe adapt it to the distance between the points? Or change the speed?)
        If checkZSafety is True, after the last sequence element the program checks if the drone is more than 0.2m above the initial_z value (see initialization of the object). If thats true it sends an extra command (lastXValue,lastYValue, 0.1+initial_z) to avoid shutting the engines off while the drone is flying high.
        """
        cf = self.scf.cf
        x = 0
        y = 0
        z = 0
        for position in sequence:
            print('Setting position {}'.format(position))

            x = position[0] + self.initial_x
            y = position[1] + self.initial_y
            z = position[2] + self.initial_z

            for i in range(10):
                cf.commander.send_position_setpoint(x, y, z, self.initial_yaw)
            time.sleep(time_between_commandos)
        if z - self.initial_z > 0.2:
            print("Sending additional position_setpoint to drone because z coordinate is {}m higher than the starting position. If you don't want that behaviour, set checkZSafety parameter to False".format(z-self.initial_z) )
            cf.commander.send_position_setpoint(x, y, self.initial_z + 0.1, self.initial_yaw)
            time.sleep(time_between_commandos)

        cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
    
    def flyToPoint(self, x, y, z, yaw=-1000,position ="relative"):
        """Sends a command to the drone to fly to a specified point.

        X,Y,Z is the coordinate the drone should fly to.
        yaw is the direction the drone should "look"
        position = relative: the x,y,z values will be added to the initial_x/y/z values.
        position = absolute: the x,y,z values will be send to the drone directly. WARNING: only use when you know what you are doing
        """
        if yaw == -1000:
            yaw = self.initial_yaw

        if position == "relative" or position == "absolute":
            if position == "relative":
                x = x + self.initial_x
                y = y + self.initial_y
                z = z + self.initial_z
                for i in range(10):
                    self.scf.cf.commander.send_position_setpoint(x,y,z,yaw)
                    time.sleep(0.1)
            if position == "absolute":
                self.scf.cf.commander.send_position_setpoint(x,y,z,yaw)
            
    def land(self):
        for i in range(20):
            self.scf.cf.commander.send_position_setpoint(self.initial_x,self.initial_y,self.initial_z+0.1,self.initial_yaw)
            time.sleep(0.1)
        self.scf.cf.commander.send_stop_setpoint()


