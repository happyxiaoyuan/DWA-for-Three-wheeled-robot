"""my_go controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
if __name__=='__main__':
      robot = Robot()
      timestep = int(robot.getBasicTimeStep())
      left_motor = robot.getDevice('left_motor')
      right_motor = robot.getDevice('right_motor')
      left_motor.setPosition(float('inf'))
      right_motor.setPosition(float('inf'))
      
      while robot.step(timestep) != -1:
          left_motor.setVelocity(-3)
          right_motor.setVelocity(-1)
      #pid_c(5,0.5,0.01,4.0])
      
    




# Enter here exit cleanup code.
