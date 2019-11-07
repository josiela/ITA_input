from Drone import Drone, BaseDrone
# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E709'

if __name__ == '__main__':
    with Drone(uri) as drone:
        drone.flyToPoint(0.8,0,0.2)
        
