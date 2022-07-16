from controller import Supervisor
from controller import DistanceSensor
import numpy

def move_around():
    
    distance = 10 - ds.getValue()
    distance_r = 10 - ds_r.getValue()
    distance_l = 10 - ds_l.getValue()
    #print(distance)
    if(distance>0.5):
        if(distance_r<0.3 and distance_l>0.3):
            lp = left_motor.getTargetPosition()
            rp = right_motor.getTargetPosition() - 3.14/16
            left_motor.setPosition(lp)
            right_motor.setPosition(rp)
            #print('1')
            return
        if(distance_r>0.3 and distance_l<0.3):
            lp = left_motor.getTargetPosition() - 3.14/16
            rp = right_motor.getTargetPosition()
            left_motor.setPosition(lp)
            right_motor.setPosition(rp)
            #print('2')
            return 
        left_motor.setPosition(left_motor.getTargetPosition() - 3.14/16)
        right_motor.setPosition(right_motor.getTargetPosition() - 3.14/16)
        #print('3')
    else:
        lp = left_motor.getTargetPosition() - 3.14/16
        rp = right_motor.getTargetPosition()
        left_motor.setPosition(lp)
        right_motor.setPosition(rp)
if __name__=="__main__":

    robot = Supervisor()
    #robot.__init__()
    node1 = robot.getFromDef('myrobot')
    # get the time step of the current world.
    timestep = 64
    nan = float('nan')
    ds = robot.getDevice('distance_sensor')
    ds.enable(timestep)
    ds_r = robot.getDevice('distance_sensor_r')
    ds_r.enable(timestep)
    ds_l = robot.getDevice('distance_sensor_l')
    ds_l.enable(timestep)
    left_motor = robot.getDevice('left_motor')
    right_motor = robot.getDevice('right_motor')
    laser_sensor = robot.getDevice('LDS-01')
    laser_sensor.enable(timestep)
    laser_sensor.enablePointCloud()
    r_p_s = robot.getDevice('r_p_s')
    l_p_s = robot.getDevice('l_p_s')
    r_p_s.enable(timestep)
    l_p_s.enable(timestep)
    left_motor.setPosition(left_motor.getTargetPosition())
    right_motor.setPosition(right_motor.getTargetPosition())
    print(laser_sensor.getHorizontalResolution())
    print(laser_sensor.getNumberOfLayers())
    n = laser_sensor.getNumberOfPoints()
    print(n)
    while robot.step(timestep) != -1:
       move_around() 
