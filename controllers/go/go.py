"""go controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
def pd_c(p,d):
    previous_chasu = 0
    intergral = 0
    while robot.step(timestep) != -1:
       mesure_d = ds.getValue()
       distance = 10 - mesure_d/100
       print(distance)
       chasu = w1.getVelocity()-w2.getVelocity()
       print(chasu)
       out = p*(mesure_d-800) + chasu-previous_chasu
       if(out>10):
           out = 10
       previous_chasu = chasu
       if(distance>2):   
           w1.setVelocity(5.0)
           w2.setVelocity(5.0)
           w3.setVelocity(5.0)
           w4.setVelocity(5.0)
       else:
           w1.setVelocity(5.0+out)
           w2.setVelocity(5.0)
           w3.setVelocity(5.0+out)
           w4.setVelocity(5.0) 
if __name__=="__main__":

    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    ds = robot.getDevice('distance_sensor')
    ds.enable(timestep)
    w1 = robot.getDevice('motor1')
    w2 = robot.getDevice('motor2')
    w1.setPosition(float('inf'))
    w1.setVelocity(0.0)
    w2.setPosition(float('inf'))
    w2.setVelocity(0.0)
    w3 = robot.getDevice('motor3')
    w4 = robot.getDevice('motor4')
    w3.setPosition(float('inf'))
    w3.setVelocity(0.0)
    w4.setPosition(float('inf'))
    w4.setVelocity(0.0)
    pd_c(0.05,0.1)
    
    # Enter here exit cleanup code.
    